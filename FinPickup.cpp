#include "stdafx.h"
#include "FinPickup.h"
#include <stdlib.h>
#include <algorithm>
#include "Fin3DAlgorithm.h"
extern FinGlobals FG;

// 开头写下的中文注释?

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 构造函数
CollideItem::CollideItem(FinPanel * pPanel, const Fin3DRay & line, const Fin3DPoint & point, char axis):
	_line(line),
	_ptPickup(point), 
	_axis(axis)
{
	_panelPtr = pPanel;
	Parse();
}

// 析构函数
CollideItem::~CollideItem()
{
}

/**
*	根据拾取点在板件上的位置生成切割方向，比如，若拾取点在板件的面5或者面6上则会生成5个切割方向
**/
std::vector<tagCutDirection> CollideItem::GetCutDirectios() const
{
	std::vector<tagCutDirection> res;
	if (_panelPtr == NULL) return res;
	Fin3DCube cubePanel = _panelPtr->Cube();
	
	tagCutDirection ctDrt;
	ctDrt.pt = _collidePoint;
	ctDrt.vt = _cutDirection;
	ctDrt.pl = _collidePlaneIndex;
	res.push_back(ctDrt);

	// 如果非正交或者碰撞点非工件的5、6面，那么只会形成一个切割
	if (!FinData::Fin3DAlgorithm::IsVectorsParallel(_cutDirection, cubePanel.F5()) || 
	FinData::Fin3DAlgorithm::IsPointOnCubeEdge(_collidePoint, cubePanel)) return res;

	// 当拾取点延长线与板件面5或面6碰撞时，要求板件的1234面都参与切割给定的立方体
	tagCutDirection p1CutItem;
	p1CutItem.vt = cubePanel.F1();
	cubePanel.GetCubePoint(3, p1CutItem.pt);
	p1CutItem.pl = 1;
	res.push_back(p1CutItem);

	tagCutDirection p2CutItem;
	p2CutItem.vt = -cubePanel.F1();
	cubePanel.GetCubePoint(1, p2CutItem.pt);
	p2CutItem.pl = 2;
	res.push_back(p2CutItem);

	tagCutDirection p3CutItem;
	p3CutItem.vt = cubePanel.F3();
	cubePanel.GetCubePoint(5, p3CutItem.pt);
	p3CutItem.pl = 3;
	res.push_back(p3CutItem);

	tagCutDirection p4CutItem;
	p4CutItem.vt = -cubePanel.F3();
	cubePanel.GetCubePoint(1, p4CutItem.pt);
	p4CutItem.pl = 4;
	res.push_back(p4CutItem);
	
	return res;
}

// 重载操作符
bool CollideItem::operator<(const CollideItem & item)
{
	if (item.Axis() != _axis) return false;
	if (_axis == 'x') {
		if (FLOAT_LT(_collidePoint.X, item.GetCollidePoint().X)) return true;
	}
	else if (_axis == 'y') {
		if (FLOAT_LT(_collidePoint.Y, item.GetCollidePoint().Y)) return true;
	}
	else if (_axis == 'z') {
		if (FLOAT_LT(_collidePoint.Z, item.GetCollidePoint().Z)) return true;
	}

	return false;
}

/**
*	解析碰撞点，切割方向
*	碰撞点为直线与板件表面接触的最靠近拾取点的那个点
*	切割方向为板件往外的方向(里侧为拾取点)
**/
bool CollideItem::Parse()
{
	if (_panelPtr == NULL) return false;
	Fin3DCube cubePanel = _panelPtr->Cube();
	struct collideInfo
	{
		bool		bCollide;			// 直线是否与平面交叉
		short		uPlaneIdx;			// 平面索引
		double		distan;				// 拾取点到平面的距离
		Fin3DPoint	ptCollide;			// 直线与平面碰撞点

		collideInfo():bCollide(false), uPlaneIdx(0),distan(0){}
	};

	short uPlaneIdx = -1;	// 离拾取点最近那个面的索引
	double minDis = 0.0;
	for (int i = 0; i < 6; i++) {
		collideInfo clInfo;
		clInfo.uPlaneIdx = i;
		Fin3DPlane pl; // 板件立方体的面
		cubePanel.GetCubePlane(i + 1, pl);
		FinData::FIN3D_RELATION re = FinData::Fin3DAlgorithm::RelationshipLinePlane(_line, pl, clInfo.ptCollide);
		if (re == FinData::EN_RELATION_INSECTION && cubePanel.IsPointInsideCube(clInfo.ptCollide)) {
			clInfo.bCollide = true;
			clInfo.distan = (clInfo.ptCollide - _ptPickup).Length();
			if (uPlaneIdx == -1 || FLOAT_LE(clInfo.distan, minDis)) { // 更新最小值
				minDis = clInfo.distan;
				uPlaneIdx = i;
				_collidePlaneIndex = i + 1;
				_collidePoint = clInfo.ptCollide;
				_cutDirection = _line.Vector();
			}
		}
	}

	return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 构造函数
FinPickup::FinPickup(const Fin3DPoint & pt, FinProduct * product) : _ptPickup(pt), _productPtr(product)
{
	_spacePtr = NULL;
	_xR = _yR = _zR = 0.0;
	_IgnoreAssemble = NULL;
	_IgnorePanel = NULL;
}

// 析构函数
FinPickup::~FinPickup()
{
	for (std::map<short, FinObj<void> *>::iterator it = _limitObj.begin(); it != _limitObj.end(); it++) {
		if (it->second != NULL) {
			delete it->second;
		}
	}
}

// 获取拾取到的立方体
Fin3DCube FinPickup::GetPickupCube()
{
	LOG_IF(INFO, FG.isLog) << __FUNCTION__;
	Fin3DCube cube; // 拾取立方体的默认尺寸为指定空间的大小
	if (_spacePtr != NULL) {
		cube = _spacePtr->Cube();
	}
	else return cube;

	// 判断是否有旋转
	bool bRotate = false;
	if (!FLOAT_EQ(_xR, 0) || !FLOAT_EQ(_yR, 0) || !FLOAT_EQ(_zR, 0)) bRotate = true;
	if (bRotate) { // 被旋转过后需要改变坐标系
		// 标准坐标系
		Fin3DVector f1st(0, 1, 0);
		Fin3DVector f3st(1, 0, 0);
		Fin3DVector f5st(0, 0, 1);
		Fin3DVector f1Rotate = FinUtilParse::RbConvertAb(f1st, _xR, _yR, _zR);
		Fin3DVector f3Rotate = FinUtilParse::RbConvertAb(f3st, _xR, _yR, _zR);
		Fin3DVector f5Rotate = FinUtilParse::RbConvertAb(f5st, _xR, _yR, _zR);
		cube = cube.ChangeCoordinate(f1Rotate, f3Rotate, f5Rotate, _xR, _yR, _zR);
	}

	// 添加默认的限制对象
	AddSpaceLimit(cube);
	
	// 获取所有碰撞对象
	if (GetCollideItems() == false) return cube;
	
	// 获取真正对拾取立方体有影响的碰撞对象数组
	std::vector<CollideItem> CutCollideItem;
	GetCollideItems(CutCollideItem);

	/*
	// 添加限定对象
	for (std::vector<CollideItem>::iterator it = CutCollideItem.begin(); it != CutCollideItem.end(); it++) {
		CollideItem & item = *it;
		Fin3DVector vtCut = item.CutDirection();
		for (int i = 0; i < 6; i++) {
			Fin3DPlane plane;
			cube.GetCubePlane(i + 1, plane);
			if (IsVectorsParallel(vtCut, plane.PVector) && FLOAT_BT(vtCut * plane.PVector, 0)) {
				FinPanelObj * pObj = new FinPanelObj(item.GetPanel(), item.GetPanel()->Cube(), item.GetPanelLimitPlane());
				AddLimitObj(i + 1, pObj);
				LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", Add CubeLimitPanel, plane:" << i + 1 << ", PanelName:"
					<< FinUtilString::WideChar2String(item.GetPanel()->name);
				break;
			}
		}
	}*/

	// 根据碰撞关系切除默认立方体
	ModifyCube(cube, CutCollideItem);

	// 处理跟切割后立方体还有碰撞的板件
	CollidePanelsModifyCube(cube);
	//TraceLimitObj();

	return cube;
}

// 调整立方体的宽深高
Fin3DPoint FinPickup::GetCubeAdjustCenter(double width, double depth, double height, Fin3DCube & cube)
{
	LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", width:" << width << ", depth:" << depth << ", height:" << height;
	if (_spacePtr == NULL) return Fin3DPoint();
	Fin3DCube cubeSpace = _spacePtr->Cube();

	struct tagVectDif
	{
		Fin3DVector				vt;				// 向量标志
		double					coorPickup;
		double					coorCenter;
		double	&				coorResCenter;

		tagVectDif(Fin3DVector vtor, double coorPic, double coorCen, double & coorRes) : vt(vtor), coorPickup(coorPic),
			coorCenter(coorCen), coorResCenter(coorRes) {}
	};

	Fin3DPoint ptResCenter;
	Fin3DPoint ptTmpCenter;
	cube.GetCubePoint(0, ptResCenter);
	cube.GetCubePoint(0, ptTmpCenter);
	tagVectDif vecMap[3] = { tagVectDif(cubeSpace.F1(), _ptPickup.Y, ptTmpCenter.Y, ptResCenter.Y), 
		tagVectDif(cubeSpace.F3(), _ptPickup.X, ptTmpCenter.X, ptResCenter.X),
		tagVectDif(cubeSpace.F5(), _ptPickup.Z, ptTmpCenter.Z, ptResCenter.Z)};

	double range = 0;
	// 宽深高修正, 若传入的长宽高数据比拾取到的立方体长宽高要小，则需要调整拾取到的立方体的长宽高
	double divWidth = width - cube.Width();
	double divDepth = depth - cube.Depth();
	double divHeight = height - cube.Height();
	
	LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", Divwidth:" << divWidth<< ", Divdepth:" << divDepth << ", Divheight:" << divHeight;
	
	if (FLOAT_LT(divWidth, 0)) {
		cube.ChangeSize(divWidth, cube.F3());
		range = abs(divWidth);

		for (int i = 0; i < 3; i++) {
			if (FinData::Fin3DAlgorithm::IsVectorsParallel(cube.F3(), vecMap[i].vt) == false) continue ;
			if (FLOAT_BT(abs(vecMap[i].coorPickup - vecMap[i].coorCenter), range / 2)) {
				if (FLOAT_LT(vecMap[i].coorPickup, vecMap[i].coorCenter)) {
					vecMap[i].coorResCenter = vecMap[i].coorResCenter - range / 2;
				}
				else {
					vecMap[i].coorResCenter = vecMap[i].coorResCenter + range / 2;
				}
			}
			else {
				vecMap[i].coorResCenter = vecMap[i].coorPickup;
			}

			break;
		}
	}
	if (FLOAT_LT(divDepth, 0)) {
		cube.ChangeSize(divDepth, cube.F1());
		range = abs(divDepth);
		for (int i = 0; i < 3; i++) {
			if (FinData::Fin3DAlgorithm::IsVectorsParallel(cube.F1(), vecMap[i].vt) == false) continue ;
			if (FLOAT_BT(abs(vecMap[i].coorPickup - vecMap[i].coorCenter), range / 2)) {
				if (FLOAT_LT(vecMap[i].coorPickup, vecMap[i].coorCenter)) {
					vecMap[i].coorResCenter = vecMap[i].coorResCenter - range / 2;
				}
				else {
					vecMap[i].coorResCenter = vecMap[i].coorResCenter + range / 2;
				}
			}
			else {
				vecMap[i].coorResCenter = vecMap[i].coorPickup;
			}

			break;
		}
	}
	if (FLOAT_LT(divHeight, 0)) {
		cube.ChangeSize(divHeight, cube.F5());
		range = abs(divHeight);
		for (int i = 0; i < 3; i++) {
			if (FinData::Fin3DAlgorithm::IsVectorsParallel(cube.F5(), vecMap[i].vt) == false) continue ;
			if (FLOAT_BT(abs(vecMap[i].coorPickup - vecMap[i].coorCenter), range / 2)) {
				if (FLOAT_LT(vecMap[i].coorPickup, vecMap[i].coorCenter)) {
					vecMap[i].coorResCenter = vecMap[i].coorResCenter - range / 2;
				}
				else {
					vecMap[i].coorResCenter = vecMap[i].coorResCenter + range / 2;
				}
			}
			else {
				vecMap[i].coorResCenter = vecMap[i].coorPickup;
			}

			break;
		}
	}

	LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", ptPicCenter(" << _ptPickup.X << "," << _ptPickup.Y << "," << _ptPickup.Z << ")";
	LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", ptResCenter(" << ptResCenter.X << "," << ptResCenter.Y << "," << ptResCenter.Z << ")";

	return ptResCenter;
}

// 用户设定距边值，所谓的距边值就是拾取到的立方体离目标边最近的那个面到外层空间对应面的那个距离
// 满足条件拾取到的立方体宽(或深或高)的一半+距边值 = 拾取到的立方体的中心点到外层空间对应面的距离
Fin3DPoint FinPickup::CubeFixing(const Fin3DCube & cubeOut, Fin3DCube & cube, short uSurface, double dst)
{
	Fin3DPoint ptRes;
	cube.GetCubePoint(0, ptRes); // 获取当前拾取到的立方体(下称拾取立方体)的中心点坐标

	if (uSurface < 3 || uSurface > 6) return ptRes;
	
	Fin3DPlane tarPlane; // 目标平面
	cubeOut.GetCubePlane(uSurface, tarPlane);

	// 如果拾取立方体与外层空间非正交，这个功能毫无意义，不必继续往下算了，返回当前中心点
	bool bTilt = true;
	Fin3DVector vtCubeTesting = cube.F1();
	if (FinData::Fin3DAlgorithm::IsVectorsParallel(vtCubeTesting, cubeOut.F1()) ||
		FinData::Fin3DAlgorithm::IsVectorsParallel(vtCubeTesting, cubeOut.F3()) ||
		FinData::Fin3DAlgorithm::IsVectorsParallel(vtCubeTesting, cubeOut.F5())) {
		bTilt = false;
	}

	if (bTilt) {
		LOG(ERROR) << __FUNCTION__ << ", Tilt Cube";
		return ptRes;
	}

	Fin3DVector vctMove; // 移动向量的方向,当拾取立方体位置不满足设置的条件时，必须要移动，移动向量的方向默认从拾取立方体中心点
						 // 指向目标平面
	switch (uSurface) {
	case 3:
		vctMove = cubeOut.F3(); break;
	case 4:
		vctMove = -cubeOut.F3(); break;
	case 5:
		vctMove = cubeOut.F5(); break;
	case 6:
		vctMove = -cubeOut.F5(); break;
	}
	
	struct tagVctDst
	{
		Fin3DVector		vt;			// 向量方向
		double			dst;		// 该方向上立方体的宽度，比如说，在F1方向上，宽度就是立方体的depth, f3为Cube.Width

		tagVctDst(const Fin3DVector & vect, double distan) { dst = distan, vt = vect; }
	};

	tagVctDst vctDstArr[3] = { tagVctDst(cube.F1(), cube.Depth()),
		tagVctDst(cube.F3(), cube.Width()),
		tagVctDst(cube.F5(), cube.Height()) };

	double lenght;   // 半截长度, 主要是为了兼容拾取立方体被旋转过的情况
	for (int i = 0; i < 3; i++) {
		if (FinData::Fin3DAlgorithm::IsVectorsParallel(vctDstArr[i].vt, vctMove)) {
			lenght = vctDstArr[i].dst / 2;
			break;
		}
	}

	// 最终立方体的位置应该满足，立方体的中心点与目标平面的距离 = 半截距离 + 设定的离边距离
	// 当前立方体中心点距离目标平面的距离
	double curDst = FinData::Fin3DAlgorithm::Distance(ptRes, tarPlane);

	double divLength = curDst - (lenght + dst);
	double n;
	try {
		n = abs(divLength) / vctMove.Length();
	}
	catch (...) {
		LOG(ERROR) << __FUNCTION__ << ", div 0";
		n = 1;
	}

	Fin3DVector vtRes(ptRes);
	if (FLOAT_BT(divLength, 0)) { // 需要往目标平面靠近
		vtRes = Fin3DVector(ptRes) + n * vctMove;
	}
	else if (FLOAT_LT(divLength, 0)) {	// 需要远离目标平面
		vtRes = Fin3DVector(ptRes) + n * (-vctMove);
	}

	ptRes.X = vtRes.x;
	ptRes.Y = vtRes.y;
	ptRes.Z = vtRes.z;

	return ptRes;
}

// 获取限制对象
std::map<short, FinObj<void> *> FinPickup::GetLimitObjs()
{
	std::map<short, FinObj<void> *> limitObjs;
	for (int i = 0; i < 6; i++) {
		std::map<short, FinObj<void> *>::iterator it = _limitObj.find(i + 1);
		if (it == _limitObj.end()) continue;
		FinObj<void> * pLimit = it->second;
		if (pLimit == NULL) continue;
		EnFreeObjType enType = pLimit->GetObjType();
		if (enType == en_FreeObj_Panel) {
			FinPanel * pPanel = static_cast<FinPanel *>(pLimit->GetInstance());
			FinPanelObj * pLimitPanel = new FinPanelObj(pPanel, pPanel->Cube(), pLimit->GetLimitPlaneIndex());
			limitObjs.insert(std::make_pair(i + 1, pLimitPanel));
		}
		else if (enType == en_FreeObj_Space) {
			FinSpace * pSpace = static_cast<FinSpace *>(pLimit->GetInstance());
			if (pSpace == NULL) continue;
			FinSpaceObj * pLimitSpace = new FinSpaceObj(pSpace, pSpace->Cube(), pLimit->GetLimitPlaneIndex());
			limitObjs.insert(std::make_pair(i + 1, pLimitSpace));
		}
		else {
			LOG(ERROR) << __FUNCTION__ << ", Error Type";
		}
	}

	return limitObjs;
}

/**
	添加六个面上默认的限制对象
*/
void FinPickup::AddSpaceLimit(Fin3DCube cube)
{
	if (_spacePtr == NULL) return;
	
	// 限制空间立方体
	Fin3DCube cubeLimitSpace = _spacePtr->Cube();

	// 遍历立方体六个面
	for (int i = 0; i < 6; i++) {
		Fin3DPlane plane;
		cube.GetCubePlane(i + 1, plane);
		for (int n = 0; n < 6; n++) {
			Fin3DPlane limitPlane;
			cubeLimitSpace.GetCubePlane(n + 1, limitPlane);
			if (FinData::Fin3DAlgorithm::IsVectorsParallel(plane.PVector, limitPlane.PVector) && FLOAT_BT(plane.PVector * limitPlane.PVector, 0)) {
				FinObj<void> * pLimit = new FinSpaceObj(_spacePtr, cubeLimitSpace, n+1);
				AddLimitObj(i + 1, pLimit);
				LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", cubePlane:" << i + 1 << ", spacePlane:" << n + 1;
				break;
			}
		}
	}
}

// 拾取过程过滤该板件
bool FinPickup::FilterPanel(FinPanel * pPanel)
{
	if (pPanel == nullptr) return true;
	if (FLOAT_LE(pPanel->geometryParams.qty, 0)) return true;
	if (pPanel->erased == true) return true;

	LOG(INFO) << __FUNCTION__ << ", PanelName:" << FinUtilString::WideChar2String(pPanel->name);

	// 非关联加工工件需要过滤掉
	if (pPanel->basePanelPtr != NULL && pPanel->basePanelPtr->isAssociMachine == false) return true;
	if (_IgnorePanel != nullptr && pPanel == _IgnorePanel) return true; // 过滤掉指定过滤板件自身
	if (_IgnoreAssemble != nullptr) {
		if (_IgnoreAssemble->IsPanelExists(pPanel->jBID)) return true; // 过滤掉指定过滤组件内板件对象
	}

	FinObj<void> * pEfcyObj = nullptr;
	if (pPanel->IsFreeObj()) {
		pEfcyObj = _productPtr->GetFreeObj(pPanel);
		LOG(INFO) << __FUNCTION__ << ", IsFreePanelObj";
	}
	else {
		FinAssemble * pTopAsm = pPanel->GetTopAsmble();
		if (pTopAsm != nullptr && pTopAsm->IsFreeObj()) {
			pEfcyObj = _productPtr->GetFreeObj(pTopAsm);
			LOG(INFO) << __FUNCTION__ << ", Create TopAsmFreeObj";
		}
	}
	if (pEfcyObj == nullptr) return false;

	FinObj<void> * pIgnorePanelObj = _productPtr->GetFreeObj(_IgnorePanel);
	if (ExistsLimitPanel(pEfcyObj, pIgnorePanelObj)) return true;

	if (_IgnoreAssemble != nullptr) {
		FinObj<void> * pIgnoreAsmObj = _productPtr->GetFreeObj(_IgnoreAssemble);
		if (ExistsLimitPanel(pEfcyObj, pIgnoreAsmObj)) return true;
	}

	return false;
}

/*
**	获取所有碰撞对象
**	直线和一块板件组成一个碰撞对象，根据该碰撞对象可得到切割方向
*/
bool FinPickup::GetCollideItems()
{
	LOG_IF(INFO, FG.isLog) << __FUNCTION__;
	if (_productPtr == nullptr || _spacePtr == nullptr) return false;

	_collideItemVectorX.clear();
	_collideItemVectorY.clear();
	_collideItemVectorZ.clear();

	// 目标空间内经过拾取点三个正交方向的射线
	Fin3DRay xPst(Fin3DVector(_spacePtr->f3), _ptPickup);	// x 正方向
	Fin3DRay xNgt = -xPst;
	Fin3DRay yPst(Fin3DVector(_spacePtr->f1), _ptPickup);
	Fin3DRay yNgt = -yPst;
	Fin3DRay zPst(Fin3DVector(_spacePtr->f5), _ptPickup);
	Fin3DRay zNgt = -zPst;


	// 遍历产品工件
	std::list<FinPanel *>::iterator itPanel;
	for (itPanel = _productPtr->panels.begin(); itPanel != _productPtr->panels.end(); itPanel++) {
		FinPanel * pPanel = *itPanel;
		if (FilterPanel(pPanel)) continue;

		LOG(INFO) << __FUNCTION__ << ", PanelCollide, PanelName:" << FinUtilString::WideChar2String(pPanel->name);

		Fin3DCube cubePanel = pPanel->Cube(); // 板件立方体
		if (FinData::Fin3DAlgorithm::CLCubeRay(cubePanel, xPst)) {
			_collideItemVectorX.push_back(CollideItem(pPanel, xPst, _ptPickup, 'x'));
		}
		if (FinData::Fin3DAlgorithm::CLCubeRay(cubePanel, xNgt)) {
			_collideItemVectorX.push_back(CollideItem(pPanel, xNgt, _ptPickup, 'x'));
		}
		if (FinData::Fin3DAlgorithm::CLCubeRay(cubePanel, yPst)) {
			_collideItemVectorY.push_back(CollideItem(pPanel, yPst, _ptPickup, 'y'));
		}
		if (FinData::Fin3DAlgorithm::CLCubeRay(cubePanel, yNgt)) {
			_collideItemVectorY.push_back(CollideItem(pPanel, yNgt, _ptPickup, 'y'));
		}
		if (FinData::Fin3DAlgorithm::CLCubeRay(cubePanel, zPst)) {
			_collideItemVectorZ.push_back(CollideItem(pPanel, zPst, _ptPickup, 'z'));
		}
		if (FinData::Fin3DAlgorithm::CLCubeRay(cubePanel, zNgt)) {
			_collideItemVectorZ.push_back(CollideItem(pPanel, zNgt, _ptPickup, 'z'));
		}
	}

	// 输出信息
	std::vector<CollideItem>::iterator itItem;
	LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", x axis Items:";
	for (itItem = _collideItemVectorX.begin(); itItem != _collideItemVectorX.end(); itItem++) {
		char szCollideItemInfo[128] = { 0 };
		sprintf_s(szCollideItemInfo, "%s, %s", __FUNCTION__, FinUtilString::WideChar2String((*itItem).GetPanel()->name).c_str());
		LOG_IF(INFO, FG.isLog) << szCollideItemInfo;
	}
	LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", y axis Items:";
	for (itItem = _collideItemVectorY.begin(); itItem != _collideItemVectorY.end(); itItem++) {
		char szCollideItemInfo[128] = { 0 };
		sprintf_s(szCollideItemInfo, "%s, %s", __FUNCTION__, FinUtilString::WideChar2String((*itItem).GetPanel()->name).c_str());
		LOG_IF(INFO, FG.isLog) << szCollideItemInfo;
	}
	LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", z axis Items:";
	for (itItem = _collideItemVectorZ.begin(); itItem != _collideItemVectorZ.end(); itItem++) {
		char szCollideItemInfo[128] = { 0 };
		sprintf_s(szCollideItemInfo, "%s, %s", __FUNCTION__, FinUtilString::WideChar2String((*itItem).GetPanel()->name).c_str());
		LOG_IF(INFO, FG.isLog) << szCollideItemInfo;
	}

	return true;
}

/**
*	获取直接影响立方体的碰撞对象数组
*	经过排序之后已经把碰撞对象按照一定的顺序排列好，碰撞对象的切割方向是从拾取点穿过板件指向外侧的
*   所以在排序好的碰撞对象数组中找到切割方向相反的碰撞对象就找到了拾取点的位置
**/ 
bool FinPickup::GetCollideItems(OUT std::vector<CollideItem> & arrary)
{
	// 排序三个方向上的碰撞对象
	std::sort(_collideItemVectorX.begin(), _collideItemVectorX.end());
	std::sort(_collideItemVectorY.begin(), _collideItemVectorY.end());
	std::sort(_collideItemVectorZ.begin(), _collideItemVectorZ.end());

	std::vector<CollideItem>::iterator itItem;
	std::vector<CollideItem>::iterator itNext;
	std::vector<CollideItem>::iterator itBegin;
	for (itItem = _collideItemVectorX.begin(); itItem != _collideItemVectorX.end(); itItem++) {
		itNext = itItem;
		itNext++;
		if (itNext == _collideItemVectorX.end()) { // 已经遍历至尾部, 说明在该数轴上所有碰撞对象都位于拾取点的一侧
													// 根据排序关系与拾取点比较看最终对拾取有影响是数组头部和尾部中哪一个对象
			LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", sameDirection";
			itBegin = _collideItemVectorX.begin();

			Fin3DPoint ptBegin = (*itBegin).GetCollidePoint();
			Fin3DPoint ptItem = (*itItem).GetCollidePoint();

			if (FLOAT_EQ(ptBegin.X, ptItem.X)) { // 头尾相等, 随便取头或者尾，目前固定尾部
				arrary.push_back(*itItem);
				break;
			}
		
			// 数组固定从小到大排序
			if (FLOAT_BE(_ptPickup.X, ptItem.X)) { // 如果拾取点大于等于尾部
				arrary.push_back(*itItem);
			}
			else { // 拾取点小于最小值，说明整个碰撞位于拾取点右侧 (x 方向上)
				arrary.push_back(*itBegin);
			}
			break;
		}
		Fin3DVector vtNext = (*itNext).CutDirection();
		Fin3DVector vtItem = (*itItem).CutDirection();
		if (FLOAT_BT(vtNext * vtItem, 0)) continue; // 过滤掉同向
		arrary.push_back(*itItem);
		arrary.push_back(*itNext);
		break;
	}
	for (itItem = _collideItemVectorY.begin(); itItem != _collideItemVectorY.end(); itItem++) {
		itNext = itItem;
		itNext++;
		if (itNext == _collideItemVectorY.end()) {
			itBegin = _collideItemVectorY.begin();

			Fin3DPoint ptBegin = (*itBegin).GetCollidePoint();
			Fin3DPoint ptItem = (*itItem).GetCollidePoint();

			if (FLOAT_EQ(ptBegin.Y, ptItem.Y)) { // 头尾相等, 随便取头或者尾，目前固定尾部
				arrary.push_back(*itItem);
				break;
			}

			// 数组固定从小到大排序
			if (FLOAT_BE(_ptPickup.Y, ptItem.Y)) { // 如果拾取点大于等于尾部
				arrary.push_back(*itItem);
			}
			else { // 拾取点小于最小值，说明整个碰撞位于拾取点右侧 (x 方向上)
				arrary.push_back(*itBegin);
			}
			break;
		}
		Fin3DVector vtNext = (*itNext).CutDirection();
		Fin3DVector vtItem = (*itItem).CutDirection();
		if (FLOAT_BT(vtNext * vtItem, 0)) continue;
		arrary.push_back(*itItem);
		arrary.push_back(*itNext);
		break;
	}
	for (itItem = _collideItemVectorZ.begin(); itItem != _collideItemVectorZ.end(); itItem++) {
		itNext = itItem;
		itNext++;
		Fin3DVector vtItem = (*itItem).CutDirection();
		if (itNext == _collideItemVectorZ.end()) {
			itBegin = _collideItemVectorZ.begin();

			Fin3DPoint ptBegin = (*itBegin).GetCollidePoint();
			Fin3DPoint ptItem = (*itItem).GetCollidePoint();

			if (FLOAT_EQ(ptBegin.Z, ptItem.Z)) { // 头尾相等, 随便取头或者尾，目前固定尾部
				arrary.push_back(*itItem);
				break;
			}

			// 数组固定从小到大排序
			if (FLOAT_BE(_ptPickup.Z, ptItem.Z)) { // 如果拾取点大于等于尾部
				arrary.push_back(*itItem);
			}
			else { // 拾取点小于最小值，说明整个碰撞位于拾取点右侧 (x 方向上)
				arrary.push_back(*itBegin);
			}		
			break;
		}
		Fin3DVector vtNext = (*itNext).CutDirection();
		if (FLOAT_BT(vtNext * vtItem, 0)) continue;
		arrary.push_back(*itItem);
		arrary.push_back(*itNext);
		break;
	}

	LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", EfficacyPanel:";
	for (itItem = arrary.begin(); itItem != arrary.end(); itItem++) {
		char szPanelName[128] = { 0 };
		sprintf_s(szPanelName, "%s, %s", __FUNCTION__, FinUtilString::WideChar2String((*itItem).GetPanel()->name).c_str());
		LOG_IF(INFO, FG.isLog) << szPanelName;
	}

	return true;
}

// 根据碰撞对象切割立方体
bool FinPickup::ModifyCube(OUT Fin3DCube & cube, std::vector<CollideItem> & arrary)
{
	LOG_IF(INFO, FG.isLog) << __FUNCTION__;
	std::vector<CollideItem>::iterator itItem;
	for (itItem = arrary.begin(); itItem != arrary.end();) {
		FinPanel * pPanel = itItem->GetPanel();
		if (pPanel == NULL) { // 碰撞对象中一定有板件
			itItem = arrary.erase(itItem);
			continue;
		}
	
		if (Cut(cube, *itItem) == false) return false;

		itItem++;
	}
	return true;
}

// 切割立方体
bool FinPickup::Cut(OUT Fin3DCube & cube, const CollideItem & item)
{
	LOG_IF(INFO, FG.isLog) << __FUNCTION__;
	std::vector<tagCutDirection> cutArr = item.GetCutDirectios();
	std::vector<tagCutDirection>::iterator itCut;
	LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", cutArr.size:" << cutArr.size();
	for (itCut = cutArr.begin(); itCut != cutArr.end(); itCut++) {
		tagCutDirection & ct = *itCut;
		LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", dir(" << (*itCut).vt.x << "," << (*itCut).vt.y << "," << (*itCut).vt.z << ")"
			<< ", pt:(" << ct.pt.X << "," << ct.pt.Y << "," << ct.pt.Z << ")";
		Fin3DVector vtCut = GetCutVector(cube, *itCut);
		char szCutVector[64] = { 0 };
		sprintf_s(szCutVector, "CutVectors(%.3lf,%.3lf,%.3lf)", vtCut.x, vtCut.y, vtCut.z);
		LOG_IF(INFO, FG.isLog) << __FUNCTION__ << "," << szCutVector;

		if (vtCut.IsZeroVector()) continue;
		cube.Cut(vtCut);

		// 添加限制对象
		for (int i = 0; i < 6; i++) {
			Fin3DPlane plane;
			cube.GetCubePlane(i + 1, plane);
			if (FinData::Fin3DAlgorithm::IsVectorsParallel((*itCut).vt, plane.PVector) && FLOAT_BT((*itCut).vt * plane.PVector, 0)) {
				FinPanelObj * pLimit = new FinPanelObj(item.GetPanel(), item.GetPanel()->Cube(), (*itCut).pl);
				AddLimitObj(i + 1, pLimit);
				break;
			}
		}
	}
	return true;
}

/**
*	获取切割向量
*	切割立方体的时候，向量肯定是从立方体的外面指向里面的，这个是前提
*	tagCutDirection 中的表示的是往外的向量，跟实际的切割方向是相反的，若tagCutDirection中的点到切割平面的向量与平面的法向量反向，则无需切割，
*	因为切割点已在立方体外部
*	根据tagCutDirection中的切割方向确定切割面，然后再求点到该平面的向量，若为同向向量, 则取反切割点到切割面的向量即为所求
**/
Fin3DVector FinPickup::GetCutVector(const Fin3DCube & cube, const tagCutDirection & dir)
{
	Fin3DVector vtRes;
	Fin3DPlane basPlane;	// 被切割的面
	if (FinData::Fin3DAlgorithm::IsVectorsParallel(cube.F1(), dir.vt)) { // 与f1平行
		if (FLOAT_BT(dir.vt * cube.F1(), 0)) { // 跟f1同向
			cube.GetCubePlane(1, basPlane);
		}
		else {
			Fin3DPlane plane2;
			cube.GetCubePlane(2, basPlane);
		}
	}
	else if (FinData::Fin3DAlgorithm::IsVectorsParallel(cube.F3(), dir.vt)) {
		if (FLOAT_BT(dir.vt * cube.F3(), 0)) {
			cube.GetCubePlane(3, basPlane);
		}
		else {
			cube.GetCubePlane(4, basPlane);
		}
	}
	else if (FinData::Fin3DAlgorithm::IsVectorsParallel(cube.F5(), dir.vt)) {
		if (FLOAT_BT(dir.vt * cube.F5(), 0)) {
			cube.GetCubePlane(5, basPlane);
		}
		else {
			cube.GetCubePlane(6, basPlane);
		}
	}

	Fin3DVector vtCut = FinData::Fin3DAlgorithm::VectorToPlane(dir.pt, basPlane); // 基准点指向切割面的向量
	if (FLOAT_BT(basPlane.Vector() * vtCut, 0)) {
		vtRes = -vtCut;
	}

	return vtRes;
}

/*
*	当立方体与板件有碰撞时，需要用板件的面1面2 或面3面4 去切割立方体
*/
bool FinPickup::CollidePanelsModifyCube(Fin3DCube & cube)
{
	if (_productPtr == NULL) {
		LOG(ERROR) << __FUNCTION__ << ", ProductPtr == null";
		return false;
	}

	std::list<FinPanel *>::iterator itProductPanel;
	for (itProductPanel = _productPtr->panels.begin(); itProductPanel != _productPtr->panels.end(); itProductPanel++) {
		FinPanel * pProductPanel = *itProductPanel;
		if (FilterPanel(pProductPanel)) continue;

		double dir = cube.F1() * pProductPanel->f1;
		if (!FinData::Fin3DAlgorithm::IsVectorsParallel(cube.F1(), pProductPanel->f1) && FLOAT_NEQ(dir, 0)) continue;  // 过滤掉非正交板件

		Fin3DCube cubePanel = pProductPanel->Cube();
		if (FinData::Fin3DAlgorithm::IsCubesCollide(cubePanel, cube) == false) continue;

		LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", PanelCollideCube, PanelName:" << FinUtilString::WideChar2String(pProductPanel->name);

		// 面1面2 和 面3面4
		for (int uPairCount = 0; uPairCount < 3; uPairCount = uPairCount+2) {
			Fin3DPlane plane1;
			Fin3DPlane plane2;
			cubePanel.GetCubePlane(uPairCount + 1, plane1);
			cubePanel.GetCubePlane(uPairCount + 2, plane2);
			// 如果拾取点不在板件面1和面2(或面3面4)所在平面的范围内
			if (FinData::Fin3DAlgorithm::IsPtBetweenPlanes(_ptPickup, plane1, plane2)) continue;
			LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", pickupPoint not between panelPlanes:" << ((uPairCount == 0) ? "12":"34");
			double d1 = FinData::Fin3DAlgorithm::Distance(_ptPickup, plane1); // 拾取点到面的距离
			double d2 = FinData::Fin3DAlgorithm::Distance(_ptPickup, plane2); // 拾取点到面的距离

			short uPanelnBasePt;			// 板件切割面上的某个顶点
			Fin3DPlane basePanelPlane;		// 板件某个切割面
			short uLimitPlane;				// 板件限制面

			LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", d1:" << d1 << ",d2:" << d2;
			// 比较拾取点到面的距离确定切割面和切割点
			if (FLOAT_BT(d1, d2)) {
				uPanelnBasePt = 1;
				basePanelPlane = plane2;
				uLimitPlane = uPairCount + 2;
			}
			else {
				uPanelnBasePt = 5;
				basePanelPlane = plane1;
				uLimitPlane = uPairCount + 1;
			}

			Fin3DVector cutVector;			// 面1或面2对立方体的切割向量
			Fin3DPoint ptCut;				// 板件切割面上任意一点，比如切割面为面2，则取板件顶点1278中的任意一个点
			cubePanel.GetCubePoint(uPanelnBasePt, ptCut);
			Fin3DVector vtPickupPlane;
			if (FLOAT_EQ(FinData::Fin3DAlgorithm::Distance(_ptPickup, basePanelPlane), 0)) {		// 如果拾取点在切割平面上，则取平面向量相反的向量，因为立方体所有的面向量都是指向外边的
				vtPickupPlane = -basePanelPlane.PVector;
			}
			else {
				vtPickupPlane = FinData::Fin3DAlgorithm::VectorToPlane(_ptPickup, basePanelPlane);	// 拾取点到板件切割面的向量
			}
			for (int i = 0; i < 6; i++) {
				Fin3DPlane plane;
				cube.GetCubePlane(i + 1, plane);
				Fin3DVector vtPickupCube = FinData::Fin3DAlgorithm::VectorToPlane(ptCut, plane);	// 板件上的切割点到立方体六个面试上的向量
				if (FLOAT_BT(vtPickupCube * vtPickupPlane, 0)) {		// 若拾取点到切割面的向量与切割面上的点到立方体某个面上的向量同向，就找到了切割向量
					cutVector = -vtPickupCube;
					LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", CubePlaneIndex:" << i + 1;
					break;
				}
			}
			LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", CutVector:" << cutVector.x << "," << cutVector.y << "," << cutVector.z;
			cube.Cut(cutVector);

			// 添加限制对象
			for (int i = 0; i < 6; i++) {
				Fin3DPlane plane;
				cube.GetCubePlane(i + 1, plane);
				if (FinData::Fin3DAlgorithm::IsVectorsParallel(cutVector, plane.PVector) && FLOAT_LE(cutVector * plane.PVector, 0)) {
					FinPanelObj * pLimit = new FinPanelObj(pProductPanel, pProductPanel->Cube(), uLimitPlane);
					AddLimitObj(i + 1, pLimit);
					LOG_IF(INFO, FG.isLog) << ", Add limitPanel, plane:" << i + 1 << ", panelName:"
						<< FinUtilString::WideChar2String(pProductPanel->name);
					break;
				}
			}
		}
	}

	return true;
}

// 添加面限制对象 uPlane 1-6
bool FinPickup::AddLimitObj(short uPlane, FinObj<void> * limitObj)
{
	std::map<short, FinObj<void> *>::iterator itExists = _limitObj.find(uPlane);
	if (itExists != _limitObj.end()) {
		FinObj<void> * pExistLimit = itExists->second;
		if (pExistLimit != NULL) delete pExistLimit;
		_limitObj.erase(itExists);
	}

	_limitObj.insert(std::make_pair(uPlane, limitObj));

	if (limitObj->GetObjType() == en_FreeObj_Panel) {
		FinPanel * pLimit = static_cast<FinPanel*>(limitObj->GetInstance());
		LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", Add Limit, planeIndex:" << uPlane << ", PanelName:"
			<< FinUtilString::WideChar2String(pLimit->name);
	}
	else if (limitObj->GetObjType() == en_FreeObj_Space) {
		FinSpace * pLimit = static_cast<FinSpace *>(limitObj->GetInstance());
		LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", Add Limit, planeIndex:" << uPlane << ", SpaceName:"
			<< FinUtilString::WideChar2String(pLimit->name);
	}
	return true;
}

void FinPickup::EraseLimitObj(short uPlane)
{
	std::map<short, FinObj<void> *>::iterator it = _limitObj.find(uPlane);
	if (it == _limitObj.end()) return;

	FinObj<void> * pLimitObj = it->second;
	_limitObj.erase(it);
	delete pLimitObj;
}

void FinPickup::EraseLimitObj(Fin3DCube & cube, const Fin3DVector & vt)
{
	short uPlaneIndex = 0;
	Fin3DVector vtArr[6] = { cube.F1(), -cube.F1(), cube.F3(), -cube.F3(), cube.F5(), -cube.F5() };
	for (int i = 0; i < 6; i++) {
		if (FinData::Fin3DAlgorithm::IsVectorsParallel(vtArr[i], vt) && FLOAT_BT(vtArr[i] * vt, 0)) {
			uPlaneIndex = i + 1;
			break;
		}
	}

	EraseLimitObj(uPlaneIndex);
}

bool FinPickup::GetLimitPlane(short uIndex, Fin3DPlane & plane)
{
	std::map<short, FinObj<void> *>::iterator it = _limitObj.find(uIndex);
	if (it == _limitObj.end()) return false;

	FinObj<void>* pLimitObj = it->second;
	Fin3DCube cubeLimit = pLimitObj->GetObjCube();
	int nLmtPlaneIdx = pLimitObj->GetLimitPlaneIndex();
	cubeLimit.GetCubePlane(nLmtPlaneIdx, plane);

	return true;
}

void FinPickup::AdjustLimitObj(Fin3DCube & cubeSpace, Fin3DCube & cubePickup, int surfaceIdx)
{
	Fin3DVector vtSurface;
	switch (surfaceIdx) {
	case 1: vtSurface = cubeSpace.F1();		break;
	case 2: vtSurface = -cubeSpace.F1();	break;
	case 3: vtSurface = cubeSpace.F3();		break;
	case 4: vtSurface = -cubeSpace.F3();	break;
	case 5: vtSurface = cubeSpace.F5();		break;
	case 6: vtSurface = -cubeSpace.F5();	break;
	}

	// 确定拾取立方体f1纵向的限制面
	Fin3DPlane planeA;
	Fin3DPlane planeB;
	if (GetLimitPlane(1, planeA) && GetLimitPlane(2, planeB)) {
		Fin3DPoint pt1;
		Fin3DPoint pt3;
		cubePickup.GetCubePoint(3, pt3);
		cubePickup.GetCubePoint(1, pt1);
		double intval1 = FinData::Fin3DAlgorithm::Distance(pt3, planeA); // 拾取立方体离限制面1的距离
		double intval2 = FinData::Fin3DAlgorithm::Distance(pt1, planeB); // 拾取立方体离限制面2的距离
		if (FLOAT_BT(intval1, 0) || FLOAT_BT(intval2, 0)) {
			if (vtSurface.IsZeroVector() == false) {
				EraseLimitObj(cubePickup, -vtSurface);
			}
			else if (FLOAT_BT(intval1, intval2)) {
				EraseLimitObj(1);
			}
			else {
				EraseLimitObj(2);
			}
		}
	}

	if (GetLimitPlane(3, planeA) && GetLimitPlane(4, planeB)) {
		Fin3DPoint pt5;
		Fin3DPoint pt1;
		cubePickup.GetCubePoint(5, pt5);
		cubePickup.GetCubePoint(1, pt1);
		double intval3 = FinData::Fin3DAlgorithm::Distance(pt5, planeA); // 拾取立方体离限制面1的距离
		double intval4 = FinData::Fin3DAlgorithm::Distance(pt1, planeB); // 拾取立方体离限制面2的距离
		if (FLOAT_BT(intval3, 0) || FLOAT_BT(intval4, 0)) {
			if (vtSurface.IsZeroVector() == false) {
				EraseLimitObj(cubePickup, -vtSurface);
			}
			else if (FLOAT_BT(intval3, intval4)) {
				EraseLimitObj(3);
			}
			else {
				EraseLimitObj(4);
			}
		}
	}
	
	if (GetLimitPlane(5, planeA) && GetLimitPlane(6, planeB)) {
		Fin3DPoint pt6;
		Fin3DPoint pt1;
		cubePickup.GetCubePoint(6, pt6);
		cubePickup.GetCubePoint(1, pt1);
		double intval5 = FinData::Fin3DAlgorithm::Distance(pt6, planeA);// 拾取立方体离限制面1的距离
		double intval6 = FinData::Fin3DAlgorithm::Distance(pt1, planeB); // 拾取立方体离限制面2的距离
		if (FLOAT_BT(intval5, 0) || FLOAT_BT(intval6, 0)) {
			if (vtSurface.IsZeroVector() == false) {
				EraseLimitObj(cubePickup, -vtSurface);
			}
			else if (FLOAT_BT(intval5, intval6)) {
				EraseLimitObj(5);
			}
			else {
				EraseLimitObj(6);
			}
		}
	}

	TraceLimitObj();
}

// 输出限制对象，调试用
void FinPickup::TraceLimitObj()
{
	for (int i=0; i<6; i++) {
		std::map<short, FinObj<void>*>::iterator it = _limitObj.find(i + 1);
		if (it == _limitObj.end()) continue;
		FinObj<void> * pObj = it->second;
		if (pObj == NULL) continue;
		if (pObj->GetObjType() == en_FreeObj_Panel) {
			FinPanel * pLimitPanel = static_cast<FinPanel *>(pObj->GetInstance());
			if (pLimitPanel != NULL) {
				LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", plane:" << it->first << ", LimitPanelName:" << FinUtilString::WideChar2String(pLimitPanel->name)
					<< ", limitPlaneIndex:" << pObj->GetLimitPlaneIndex();
			}
		}
		else if (pObj->GetObjType() == en_FreeObj_Space) {
			FinSpace * pLimitSpace = static_cast<FinSpace *>(pObj->GetInstance());
			if (pLimitSpace != NULL) {
				LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", plane:" << it->first << ", LimitSpaceName:" << FinUtilString::WideChar2String(pLimitSpace->name)
					<< ", limitPlaneIndex:" << pObj->GetLimitPlaneIndex();
			}
		}
		else {
			LOG_IF(INFO, FG.isLog) << __FUNCTION__ << ", LimitType:" << pObj->GetObjType();
		}
	}
}

bool FinPickup::ExistsLimitPanel(FinObj<void> * pFreeObj, FinObj<void> * pTarFreeObj)
{
	if (pTarFreeObj == nullptr || pFreeObj == nullptr) return false;

	FinObj<void> * pObj = pFreeObj;
	if (pFreeObj->GetObjType() == en_FreeObj_Panel) {
		FinPanel * pCenter = static_cast<FinPanel*>(pFreeObj->GetInstance());
		FinAssemble * pTopAsm = pCenter->GetTopAsmble();
		if (pTopAsm != nullptr) {
			FinObj<void> * pFreeAsmObj = _productPtr->GetFreeObj(pTopAsm);
			if (pFreeObj != nullptr) {
				pObj = pFreeAsmObj;
			}
		}
	}

	LOG(INFO) << __FUNCTION__ << ", FilterPanelName:" << FinUtilString::WideChar2String(pFreeObj->Name())
		<< ", TarObjName:" << FinUtilString::WideChar2String(pTarFreeObj->Name());

	// 遍历六个面的限制对象
	for (int i = 1; i <= 6; i++) {
		FinObj<void> * pLimitObj = pObj->GetPlaneLimitObj(i);
		if (pLimitObj != nullptr) {
			FinObj<void> * pNext = nullptr;
			LOG(INFO) << __FUNCTION__ << ", planeIdx:" << i << ", limitObjName:" << FinUtilString::WideChar2String(pLimitObj->Name());
			if (*pLimitObj == *pTarFreeObj) return true;

			if (pLimitObj->GetObjType() == en_FreeObj_Panel) {
				FinPanel * pPanel = static_cast<FinPanel *>(pLimitObj->GetInstance());
				if (pPanel != nullptr) {
					if (pPanel->IsFreeObj()) pNext = _productPtr->GetFreeObj(pPanel);
					FinAssemble * pTpAsm = pPanel->GetTopAsmble();
					if (pTpAsm != nullptr) {
						if (pTpAsm->IsFreeObj()) {
							FinObj<void> * pFreeAsm = _productPtr->GetFreeObj(pTpAsm);
							if (pFreeAsm != nullptr) {
								if (*pFreeAsm == *pTarFreeObj) return true;
								pNext = pFreeAsm;
							}
						}
					}
				}
			}

			if (ExistsLimitPanel(pNext, pTarFreeObj)) return true;
		}
	}
	
	return false;
}
