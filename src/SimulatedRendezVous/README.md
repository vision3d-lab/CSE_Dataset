# Simulated Renderzvous

C-SLAM evaluation 을 위해 로봇 간 communication 상태를 가상으로 조정하는 방법에 대해 논합니다.
본 데이터셋은 constant communication을 가정하고 있으나, 아래의 모듈을 활용하면 가상으로 connection을 허용하거나 허용하지 않을 수 있습니다.

'''cpp
// https://github.com/lajoiepy/cslam/blob/32527430ef271064d468f442a3ab754917abb011/src/back_end/utils/simulated_rendezvous.cpp#L51
를 참고한 코드입니다.
'''

## How to use
