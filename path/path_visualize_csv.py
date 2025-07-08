import pandas as pd
import matplotlib.pyplot as plt

# CSV 파일을 읽어옵니다.
# 'data.csv' 파일이 코드와 같은 폴더에 있어야 합니다.
try:
    df = pd.read_csv('path_corrected_3.csv')
except FileNotFoundError:
    print("'data.csv' 파일을 찾을 수 없습니다. 파일명과 경로를 확인해주세요.")
    exit()

# 10개 행마다 하나의 데이터를 선택합니다. (0, 10, 20, ... 번째 인덱스)
# df[::10]는 처음부터 끝까지 10칸씩 건너뛰며 데이터를 선택하는 구문입니다.
df_sampled = df[::40]

# 시각화 설정
plt.figure(figsize=(10, 8)) # 그래프 크기 조절

# 전체 데이터 포인트를 회색의 작은 점으로 표시 (선택 사항)
plt.scatter(df['X'], df['Y'], color='lightgray', s=10, label='전체 데이터')

# 10개마다 샘플링한 데이터를 빨간색의 큰 점으로 표시
plt.scatter(df_sampled['X'], df_sampled['Y'], color='red', s=5, label='10개마다 선택된 데이터')

# 그래프 제목 및 라벨 설정
plt.title('CSV 데이터 시각화 (10개마다 점 찍기)')
plt.xlabel('X 좌표')
plt.ylabel('Y 좌표')
plt.legend() # 범례 표시
plt.grid(True) # 그리드 표시
plt.axis('equal') # X, Y 축 비율을 동일하게 설정

# 그래프 보여주기
plt.show()