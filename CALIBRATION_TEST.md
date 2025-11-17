# FTM 캘리브레이션 테스트 가이드

## 빌드 및 플래시

### Beacon 빌드
```bash
cd /Users/air/Desktop/guma2/beacon
idf.py build
idf.py flash monitor
```

## 테스트 절차

### 1. 초기 테스트 (캘리브레이션 검증)
알려진 거리에 beacon을 배치하고 측정:

| 실제 거리 | 기대 측정값 (0.2 팩터) |
|----------|---------------------|
| 0.5m     | ~0.6m              |
| 1.0m     | ~0.8-1.2m          |
| 1.5m     | ~1.2-1.5m          |
| 2.0m     | ~1.6-2.0m          |

### 2. 로그 확인
측정 로그에서 다음 정보를 확인:

```
FTM sample X: RTT=XXXXX ps (XX.XXns), raw=X.XX m, calibrated=X.XX m - VALID
```

- **RTT (ps)**: 원본 Round-Trip Time
- **raw**: 캘리브레이션 전 거리
- **calibrated**: 캘리브레이션 후 거리 (이 값이 실제 거리와 비교됨)

### 3. 캘리브레이션 팩터 미세 조정

측정 결과를 기반으로 팩터 조정:

**팩터 계산:**
```
새로운 팩터 = 현재 팩터 × (실제 거리 / 측정 거리)
```

**예시:**
- 실제: 1.0m, 측정: 1.2m
- 새 팩터 = 0.20 × (1.0 / 1.2) = 0.167

**코드 수정 위치:**
`/Users/air/Desktop/guma2/beacon/main/main.c:57`
```c
#define FTM_CALIBRATION_FACTOR 0.20f  // 여기를 조정
```

### 4. Variance threshold 조정 (필요시)

재시도가 너무 많이 발생하면 threshold 증가:

**코드 수정 위치:**
`/Users/air/Desktop/guma2/beacon/main/main.c:63`
```c
#define MAX_VARIANCE_THRESHOLD 0.10f  // 0.15나 0.20으로 증가 가능
```

## 다양한 거리에서 정확도 테스트

| 거리 범위 | 목표 정확도 |
|---------|----------|
| 0.5-2m  | ±0.3m    |
| 2-5m    | ±0.5m    |
| 5-10m   | ±1.0m    |

## Gateway는 수정 불필요

Gateway는 FTM responder 역할만 하며:
- Beacon이 계산한 거리를 받아서 Kalman 필터 적용
- 캘리브레이션된 거리가 자동으로 전달됨
- Gateway 코드 수정 불필요

## 문제 해결

### 측정값이 여전히 부정확한 경우

1. **더 많은 측정 데이터 수집**
   - 3-5개의 다른 거리에서 측정
   - 평균 오차율 계산

2. **비선형 캘리브레이션 고려**
   - 거리별로 다른 팩터가 필요한 경우
   - 거리 기반 lookup table 구현

3. **환경 요인 확인**
   - 금속 물체, 벽 반사 등이 영향
   - 가시선(Line of Sight) 확보
