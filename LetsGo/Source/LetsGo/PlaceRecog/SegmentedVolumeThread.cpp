#include "SegmentedVolumeThread.h"


FSegmentedVolumeThread::FSegmentedVolumeThread(ASegmentedVolumeMgr* _segVolMgr, SlamDatabase* _slamDB, SegmentedVolume* _SegVolume)
{
	segVolMgr = _segVolMgr;
	slamDB = _slamDB;
	SegVolume = _SegVolume;
}

bool FSegmentedVolumeThread::Init()
{
	bClusterStart = false;
	bClusterDone = false;
	bStopThread = false;
	return true;
}

uint32 FSegmentedVolumeThread::Run()
{
	int cnt = 0;
	while (!bStopThread)
	{
		FPlatformProcess::Sleep(0.1);
		if (bClusterStart)
		{
			bClusterStart = false;
			clusterPointCloud();
			bClusterDone = true;
		}
	}
	return 0;
}

void FSegmentedVolumeThread::Stop()
{

}

//#########################################
// Segmentation
//#########################################
void FSegmentedVolumeThread::clusterPointCloud()
{
	///�ڽ� ���� ��ǥ�� ��ȯ��� ���ϱ�
	bool isVolumeInitialized = getBoundingBoxOrientedCoord();


	///���� ��������
	Eigen::Vector3f unitSize = boxProjHalfSize / 20;
	pcl::PointCloud<pcl::PointXYZI> pcl_pc_in_b;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
	for (int i = 0; i < slamDB->curFrameInlierLandmarks.size(); i++)
	{
		Eigen::Vector3f pconv = convertCoordPoint(FVec2EigenVec(slamDB->curFrameInlierLandmarks[i]));


		//�ڽ� �ٱ����� ���� �� ����
		if (pconv[0] <= -boxProjHalfSize[0] - unitSize[0] || pconv[0] > boxProjHalfSize[0] + unitSize[0])
			continue;
		if (pconv[1] <= -boxProjHalfSize[1] - unitSize[1] || pconv[1] > boxProjHalfSize[1] + unitSize[1])
			continue;
		if (pconv[2] <= -boxProjHalfSize[2] - unitSize[2] || pconv[2] > boxProjHalfSize[2] - unitSize[2])
			continue;
		/*
		//�ڽ��� ������ ���׸�Ʈ�� ������ �� �߰�
		if (pconv[0] <= -boxProjHalfSize[0] + unitDist || pconv[0] > boxProjHalfSize[0] - unitDist)
			continue;
		if (pconv[1] <= -boxProjHalfSize[1] + unitDist || pconv[1] > boxProjHalfSize[1] - unitDist)
			continue;
		if (pconv[2] <= -boxProjHalfSize[2] + unitDist || pconv[2] > boxProjHalfSize[2] - unitDist)
			continue;
		*/

		pcl::PointXYZI p;
		p.x = pconv[0];
		p.y = pconv[1];
		p.z = pconv[2];
		p.intensity = 1.0f;

		pcl_pc_in_b.push_back(p);
	}
	*pcl_pc_in = pcl_pc_in_b;


	///�Ķ���� ����
	// regions_dist original = 500
	// tolerance original = 90
	float boxDist = sqrt(boxProjHalfSize[0] * boxProjHalfSize[0] + boxProjHalfSize[1] * boxProjHalfSize[1] + boxProjHalfSize[2] * boxProjHalfSize[2]);
	int regions_dist = (int)(boxDist / 5);
	float tolerance = regions_dist * GET_MAINST->clusteringToleranceMult;
	PRINTF("regions_dist: %d  /  tolerance: %f", regions_dist, tolerance);


	///Ŭ�����͸� ����
	std::vector<std::vector<Eigen::Vector3f>> boxes = SegVolume->Clustering(
		pcl_pc_in, unitSize,
		-10000, 10000, GET_MAINST->clusteringClusterSizeMin, 2200000,								// z_axis_min_, z_axis_max_, cluster_size_min_, cluster_size_max_,
		regions_dist, tolerance, GET_MAINST->clusteringDistFilterWeight, GET_MAINST->clusteringToleranceWeight);	// regions_dist, tolerance_, dist_filter_weight_, tolerance_weight_


	///�ڽ��� �ٴڸ��� ������ �ϴܿ� ������ ��� ground surface�� ����
	Eigen::Vector3f groundCenter = convertCoordPoint(convertCoordplaneCenter);
	float thresZ = groundCenter[2] + boxProjHalfSize[2] * 3 / 4;
	for (int i = 0; i < boxes.size(); i++)
	{
		Eigen::Vector3f boxGroundPlaneCenter = (boxes[i][0] + boxes[i][1] + boxes[i][2] + boxes[i][3]) / 4;
		if (boxGroundPlaneCenter[2] < thresZ)
		{
			boxes[i][0][2] = groundCenter[2];
			boxes[i][1][2] = groundCenter[2];
			boxes[i][2][2] = groundCenter[2];
			boxes[i][3][2] = groundCenter[2];
		}
	}


	///�̳� �ڽ� ����
	float unitDist = sqrt(unitSize[0] * unitSize[0] + unitSize[1] * unitSize[1] + unitSize[2] * unitSize[2]);
	Eigen::Vector3f unitSizeMean(unitDist, unitDist, unitDist);
	for (int idx = 0; idx < boxes.size();)
	{
		bool isMerged = false;
		Eigen::Vector3f baseMin = boxes[idx][0];
		Eigen::Vector3f baseMax = boxes[idx][6];
		for (int i = 0; i < boxes.size(); i++)
		{
			if (i == idx)
				continue;

			Eigen::Vector3f targetMin = boxes[i][0] - unitSizeMean;
			Eigen::Vector3f targetMax = boxes[i][6] + unitSizeMean;

			if (baseMin[0] > targetMin[0] && baseMin[1] > targetMin[1] && baseMin[2] > targetMin[2])
			{
				if (baseMax[0] < targetMax[0] && baseMax[1] < targetMax[1] && baseMax[2] < targetMax[2])
				{
					isMerged = true;
					boxes.erase(boxes.begin() + idx);
					break;
				}

			}
		}
		if (!isMerged)
			idx++;
	}

	///���� �ڽ� ����
	Eigen::Vector3f boxMin = groundCenter - boxProjHalfSize + Eigen::Vector3f(0, 0, boxProjHalfSize[2]);
	Eigen::Vector3f boxMax = groundCenter + boxProjHalfSize + Eigen::Vector3f(0, 0, boxProjHalfSize[2]);
	for (int idx = 0; idx < boxes.size();)
	{
		bool isRemove = false;
		Eigen::Vector3f baseMin = boxes[idx][0];
		Eigen::Vector3f baseMax = boxes[idx][6];

		if (baseMin[0] < boxMin[0]) baseMin[0] = boxMin[0];
		if (baseMin[1] < boxMin[1]) baseMin[1] = boxMin[1];
		if (baseMin[2] < boxMin[2]) baseMin[2] = boxMin[2];
		if (baseMax[0] > boxMax[0]) baseMax[0] = boxMax[0];
		if (baseMax[1] > boxMax[1]) baseMax[1] = boxMax[1];
		if (baseMax[2] > boxMax[2]) baseMax[2] = boxMax[2];

		Eigen::Vector3f diff = baseMax - baseMin;
		//float dist = sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]);
		//if (dist < unitDist * 3)
		float mult = 1.0;
		if (diff[0] < unitDist * mult || diff[1] < unitDist * mult || diff[2] < unitDist * mult)
		{
			isRemove = true;
			boxes.erase(boxes.begin() + idx);
		}

		if (!isRemove)
			idx++;
	}

	///õ�忡 ���� �ڽ� ����
	float thresCeiling = groundCenter[2] + boxProjHalfSize[2] * 5 / 3;
	for (int idx = 0; idx < boxes.size();)
	{
		bool isRemove = false;

		if (boxes[idx][5][2] > thresCeiling)
		{
			isRemove = true;
			boxes.erase(boxes.begin() + idx);
		}

		if (!isRemove)
			idx++;
	}

	///��ǥ�� ��ȯ �� ����
	clusteringBoxes.clear();
	clusteringBoxes.resize(boxes.size());
	clusteringBoxPlaneStats.clear();
	clusteringBoxPlaneStats.resize(boxes.size());
	for (int i = 0; i < boxes.size(); i++)
	{
		//�ڽ� �𼭸����� ����
		clusteringBoxes[i].resize(8);
		for (int j = 0; j < 8; j++)
		{
			Eigen::Vector3f boxP = reconvertCoordPoint(boxes[i][j]);
			clusteringBoxes[i][j] = EigenVec2FVec(boxP);
		}

		//�ڽ� ������� ����
		for (int j = 0; j < 6; j++)
		{
			BoxPlaneStat* st = new BoxPlaneStat();
			for (int k = 0; k < 4; k++)
				st->cornerIndices[k] = GET_MAINST->bbMgr->planePointIndices[j][k];

			st->boxIdx = i;
			st->score = 0;
			st->depth = getdistOfTwoPoint(clusteringBoxes[i][GET_MAINST->bbMgr->planeDepthIndices[j][0]],
				clusteringBoxes[i][GET_MAINST->bbMgr->planeDepthIndices[j][1]]);


			clusteringBoxPlaneStats[i].push_back(st);
		}
	}
	setBoxPlaneTypes();

}

void FSegmentedVolumeThread::setBoxPlaneTypes()
{
	FVector camPos = GET_MAINST->slamMgr->curFrameCameraPos;

	for (int boxIdx = 0; boxIdx < clusteringBoxPlaneStats.size(); boxIdx++)
	{
		for (int i = 0; i < 6; i++)
		{
			BoxPlaneStat* st = clusteringBoxPlaneStats[boxIdx][i];

			//��� 4�� ��ġ ȹ��
			FVector rects_w[4];
			for (int j = 0; j < 4; j++)
				rects_w[j] = clusteringBoxes[boxIdx][st->cornerIndices[j]];

			//��� �Ӽ� ȹ��
			FVector center = (rects_w[0] + rects_w[1] + rects_w[2] + rects_w[3]) / 4;
			FVector vecRight = -(rects_w[0] - rects_w[1]);
			FVector vecUp = -(rects_w[3] - rects_w[0]);
			FVector vecForward = -FVector::CrossProduct(vecRight, vecUp);
			vecRight.Normalize(); vecUp.Normalize(); vecForward.Normalize();

			//ī�޶� �Ӽ� ȹ��
			FVector camLookVec = center - camPos;
			camLookVec.Normalize();

			//�ڻ��� ���絵 ����
			float cosSim = FVector::DotProduct(camLookVec, vecForward); //������ ���̰� 1�̱� ������ �и� ����
			if (cosSim <= 0.2)
				//���絵�� 0 ������ ��� ���̴� ���
				st->planeType = SEG_VISIBLE;
			else
				//���絵�� 0 ���� Ŭ ��� ������ �ʴ� ���(���� ������ ���� ������ ���)
				st->planeType = SEG_NOT_VISIBLE;

		}
	}
}

bool FSegmentedVolumeThread::getBoundingBoxOrientedCoord()
{
	if (GET_MAINST->bbMgr->bestBoxPlaneStats.size() != 6)
		return false;

	int* groundPlaneIdcs = GET_MAINST->bbMgr->bestBoxPlaneStats[GET_MAINST->bbMgr->groundPlaneIdx]->cornerIndices;
	double groundUpDepth = GET_MAINST->bbMgr->bestBoxPlaneStats[GET_MAINST->bbMgr->groundPlaneIdx]->depth;

	//���� ���
	Eigen::Vector3f  vec_a = (FVec2EigenVec(GET_MAINST->bbMgr->bestBoxRects[groundPlaneIdcs[1]]) - FVec2EigenVec(GET_MAINST->bbMgr->bestBoxRects[groundPlaneIdcs[0]]));
	Eigen::Vector3f  vec_b = (FVec2EigenVec(GET_MAINST->bbMgr->bestBoxRects[groundPlaneIdcs[2]]) - FVec2EigenVec(GET_MAINST->bbMgr->bestBoxRects[groundPlaneIdcs[0]]));
	Eigen::Vector3f  planeNormVec = vec_b.cross(vec_a).normalized();
	Eigen::Vector3f  planeCenter = (FVec2EigenVec(GET_MAINST->bbMgr->bestBoxRects[groundPlaneIdcs[0]]) + FVec2EigenVec(GET_MAINST->bbMgr->bestBoxRects[groundPlaneIdcs[1]])
		+ FVec2EigenVec(GET_MAINST->bbMgr->bestBoxRects[groundPlaneIdcs[2]]) + FVec2EigenVec(GET_MAINST->bbMgr->bestBoxRects[groundPlaneIdcs[3]])) / 4;

	//���� ���� ���Ϸ��� ���
	double normRollRad = EigenVec2FVec(planeNormVec).Rotation().Roll * PI / 180;
	double normPitchRad = -EigenVec2FVec(planeNormVec).Rotation().Pitch * PI / 180;
	double normYawRad = EigenVec2FVec(planeNormVec).Rotation().Yaw * PI / 180;

	//���� ���� ȸ����� ���
	Eigen::Matrix3f normYawMat, normPitchMat, normRollMat;
	Eigen::Matrix3f normRotationMat, normRotationMatInv;
	normRollMat <<
		1, 0, 0,
		0, cos(normRollRad), -sin(normRollRad),
		0, sin(normRollRad), cos(normRollRad);
	normPitchMat <<
		cos(normPitchRad), 0, sin(normPitchRad),
		0, 1, 0,
		-sin(normPitchRad), 0, cos(normPitchRad);
	normYawMat <<
		cos(normYawRad), -sin(normYawRad), 0,
		sin(normYawRad), cos(normYawRad), 0,
		0, 0, 1;
	normRotationMat = normRollMat;
	normRotationMat = normPitchMat * normRotationMat;
	normRotationMat = normYawMat * normRotationMat;
	normRotationMatInv = normRotationMat.inverse();

	Eigen::Vector3f vec_a_proj = (normRotationMatInv * vec_a).normalized();
	double vec_a_rad = acos(vec_a_proj[1]);//< vec_a_proj�� (0,1,0)�� ���̰� ����
	Eigen::Matrix3f vecARollMat, vecPitchMat;
	vecPitchMat <<
		cos(-PI / 2), 0, sin(-PI / 2),
		0, 1, 0,
		-sin(-PI / 2), 0, cos(-PI / 2);
	if (vec_a_proj[2] >= 0)
	{
		vecARollMat <<
			1, 0, 0,
			0, cos(-vec_a_rad), -sin(-vec_a_rad),
			0, sin(-vec_a_rad), cos(-vec_a_rad);
	}
	else
	{
		vecARollMat <<
			1, 0, 0,
			0, cos(vec_a_rad), -sin(vec_a_rad),
			0, sin(vec_a_rad), cos(vec_a_rad);
	}
	normRotationMatInv = vecPitchMat * vecARollMat * normRotationMatInv;
	vec_a_proj = normRotationMatInv * vec_a;

	convertCoordPlaneMat = normRotationMatInv;
	convertCoordplaneCenter = planeCenter;

	//�ڽ��� Half Size ���ϱ�
	boxProjHalfSize[0] = getdistOfTwoPoint(GET_MAINST->bbMgr->bestBoxRects[groundPlaneIdcs[1]], GET_MAINST->bbMgr->bestBoxRects[groundPlaneIdcs[2]]) / 2;
	boxProjHalfSize[1] = getdistOfTwoPoint(GET_MAINST->bbMgr->bestBoxRects[groundPlaneIdcs[0]], GET_MAINST->bbMgr->bestBoxRects[groundPlaneIdcs[1]]) / 2;
	boxProjHalfSize[2] = groundUpDepth / 2;

	return true;
}
Eigen::Vector3f FSegmentedVolumeThread::convertCoordPoint(Eigen::Vector3f point)
{
	Eigen::Vector3f pointConv = convertCoordPlaneMat * (point - convertCoordplaneCenter) - Eigen::Vector3f(0, 0, boxProjHalfSize[2]);
	return pointConv;
}
Eigen::Vector3f FSegmentedVolumeThread::reconvertCoordPoint(Eigen::Vector3f point)
{
	Eigen::Vector3f pointConv = (convertCoordPlaneMat.inverse() * (point + Eigen::Vector3f(0, 0, boxProjHalfSize[2]))) + convertCoordplaneCenter;
	return pointConv;
}

//#########################################
// ��Ÿ
//#########################################
double FSegmentedVolumeThread::getdistOfTwoPoint(FVector a, FVector b)
{
	return FMath::Sqrt((a.X - b.X)*(a.X - b.X) + (a.Y - b.Y)*(a.Y - b.Y) + (a.Z - b.Z)*(a.Z - b.Z));
}
Eigen::Vector3f FSegmentedVolumeThread::FVec2EigenVec(FVector vec)
{
	return Eigen::Vector3f(vec.X, vec.Y, vec.Z);
}
FVector FSegmentedVolumeThread::EigenVec2FVec(Eigen::Vector3f  vec)
{
	return FVector(vec[0], vec[1], vec[2]);
}
