# Reeds-Shepp
## Reeds-Shepp simple car
![3-Figure1-1](https://user-images.githubusercontent.com/49792776/83969140-2fcd3980-a909-11ea-89d6-8ecfe8b0c740.png)  
좌표와 각도로 표시되는 단순한 차 모델을 좌표에 랜덤으로 샘플링하고 planner 알고리즘에 따라 최단거리를 연결합니다.  
목표에서 step size 보다 가까운 샘플이 있다면 샘플링을 종료하고 경로를 생성합니다.  

## reeds_shepp_planner.py  
"Optimal Paths for a Car that Goes Both Forwards and Backwards" by J. A. REEDS AND L. A. SHEPP을 기반으로 샘플들을 이어서 최적의 경로를 생성합니다.
* CSC, CCC, SCS 세가지 방법의 이동거리를 계산하고 최단거리를 선택합니다.
* 최적의 경로를 설정하고 그 사이를 잇는 새로운 샘플을 생성합니다.
  
![low_goal_sample_rate](https://user-images.githubusercontent.com/49792776/83969347-7b341780-a90a-11ea-8b53-89d1d60f722a.PNG)  
![high_goal_sample_rate](https://user-images.githubusercontent.com/49792776/83969350-7bccae00-a90a-11ea-94c0-12d9979747db.PNG)  

샘플 생성을 의도적으로 더 진행한다면 더 최적화된 경로를 도출해낼 수 있습니다.
