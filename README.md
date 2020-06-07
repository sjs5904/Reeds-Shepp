# Reeds-Shepp
![3-Figure1-1](https://user-images.githubusercontent.com/49792776/83969140-2fcd3980-a909-11ea-89d6-8ecfe8b0c740.png)  
좌표와 각도로 표시되는 단순한 차 모델을 좌표에 랜덤으로 샘플링하고 planner 알고리즘에 따라 최단거리를 연결합니다.  
목표에서 step size 보다 가까운 샘플이 있다면 샘플링을 종료하고 경로를 생성합니다.  

## reeds_shepp_planner.py  
* tries to find paths with different types of method (CSC, CCC, SCS). Generate curves and straight line, then generate (x,y,theta) that interpolates paths.  
* returns path with minimum length  
  
![low_goal_sample_rate](https://user-images.githubusercontent.com/49792776/83969347-7b341780-a90a-11ea-8b53-89d1d60f722a.PNG)  
![high_goal_sample_rate](https://user-images.githubusercontent.com/49792776/83969350-7bccae00-a90a-11ea-94c0-12d9979747db.PNG)  
