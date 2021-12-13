# 소개
오늘날의 증강 게임은 더욱 섬세한 상호작용과 시각적 표현을 위해 주변 공간의 구체적인 기하 정보가 필요합니다.
기존의 실내 공간 인식 방법은 점군의 밀도와 노이즈에 취약하고, 미완성된 기하 정보만을 검출합니다.
따라서 주요 평면의 완전한 인식이 어렵고, 실세계 속 객체의 부피나 구체적인 영역 등은 파악할 수 없습니다.
직육면체 형태의 공간의 특성을 고려한 공간 인식은 대부분의 실내 공간에서 활용할 수 있습니다.
직육면체 공간에서 실내의 특성을 고려한 기하 검출 방법을 통해 많은 영역을 비교적 정확하게 검출할 수 있습니다.

<div align="center">
  <img class="scalezoom_small" src="https://user-images.githubusercontent.com/32832618/145800795-80c31fa3-689f-4b04-8698-baaa697110a1.png" height="300">
  <img class="scalezoom_small" src="https://user-images.githubusercontent.com/32832618/145800598-1699f01c-c889-409d-8f7f-f3ce92e3cc15.jpg" height="300">
</div>

본 연구 프로그램은 직육면체 형태의 실내 공간에서 증강 게임 공간을 구성하는 방법을 제안합니다.<br>
제안 알고리즘은 빠른 공간 인식과 응용 콘텐츠 개발 다양화의 폭을 넓히기 위해 구면 파노라마 비디오를 입력으로 하여 모든 시야각을 활용합니다.<br>
제안 알고리즘은 기존의 평면 검출 기반의 접근 방법에서 벗어나 Oriented Bounding Box (OBB) 검출을 통한 접근 방법으로 실내 표면 기하를 검출합니다.<br>
또한, 제안 알고리즘은 계층적 유클리드 군집화와 군집 간소화 및 조정을 통해 다중 객체 영역을 검출합니다.<br>
제안하는 검출 알고리즘은 점군의 밀도 변화에 강건하고, 실내 표면이나 객체에 대한 완전한 기하 정보를 검출할 수 있습니다.<br>

<div align="center">
  <img class="scalezoom_small" src="https://user-images.githubusercontent.com/32832618/145800609-b1bf6641-7290-46bd-83eb-c507f1ee67fe.png" height="400">
</div>

# 상세 설명
<div align="center">
  <img class="scalezoom_small" src="https://user-images.githubusercontent.com/32832618/145800944-9f5ae1d9-e6c1-413a-8a51-250aafbb4c44.png" height="250">
  <img class="scalezoom_small" src="https://user-images.githubusercontent.com/32832618/145800947-a548d94e-8c31-4373-9758-e7955174e511.png" height="250">
</div>
실내 기하 검출 과정은 전처리, 실내 표면 직육면체 검출, 다중 객체 영역 검출의 3개의 주요 과정으로 구성됩니다.
먼저, 전처리 과정에서는 구면 파노라마 기반 Vision SLAM을 통해 실시간 점군 데이터를 획득하고, KDTree를 통해 이웃점과 거리가 먼 이상점을 제거합니다.
다음으로, 실내 표면 직육면체 검출 과정에서는 점군에 대한 최적의 OBB를 획득하고, 조정 과정을 통해 표면 기하를 갱신합니다.
마지막으로, 다중 객체 영역 검출 과정에서는 실내 점군에 대한 유클리드 클러스터링을 통해 영역을 산정하고, 조정 과정을 통해 객체 영역을 획득합니다.

기하 검출 과정 이후에는 가상세계 구조물 구축, 다면 텍스처 투영 과정으로 구성됩니다.
가상세계 구조물 구축 과정에서는 검출하였던 기하 정보를 가상세계에 반영하고, 실세계와 동일 시점의 플레이어를 가상세계에 배치합니다.
다면 텍스처 투영 과정에서는 기하 구조물의 각 면에 부분적인 핀홀 투영을 통해 최종적인 1인칭 합성 장면을 획득합니다.
더욱 자세한 내용은 하단의 버튼을 통해 확인하실 수 있습니다.

# letsgo-ar-space-generation
This is AR workspace generation method using volume detection based on point clouds of spherical panorama

# If you want to compile this project
- It is developed using UE 4.19.2 Environment

- Compile with visual studio 2017 in "Development Editor" + "Win64" mode.
- Change Pathes String in mainST.cpp(this is main singletone) in C++ script then build.
- Before c++ build, delete solution file(LetsGo.sln), then execute "Generate Visual Studio Project File" in .uproject.

- Before testing program, you need to store your data in LetsGo/inputs/...
- We commite without input videos because of size. so add videos of spherical panorama
