import pandas as pd
import matplotlib.pyplot as plt

# CSV 파일을 읽어옵니다.
# 'path_xy.csv' 파일이 코드와 같은 폴더에 있어야 합니다.
try:
    df = pd.read_csv('path_xy.csv')
except FileNotFoundError:
    print("'path_xy.csv' 파일을 찾을 수 없습니다. 파일명과 경로를 확인해주세요.")
    exit()

# 40개 행마다 하나의 데이터를 선택합니다. (0, 40, 80, ... 번째 인덱스)
df_sampled = df[::40]

# 시각화 설정
fig, ax = plt.subplots(figsize=(10, 8)) # 그래프 크기 조절 및 figure, axes 객체 얻기

# 전체 데이터 포인트를 회색의 작은 점으로 표시 (선택 사항)
ax.scatter(df['X'], df['Y'], color='lightgray', s=10, label='전체 데이터')

# 40개마다 샘플링한 데이터를 빨간색의 큰 점으로 표시
# 이 점들을 이동 가능하게 할 것입니다.
draggable_points = ax.scatter(df_sampled['X'], df_sampled['Y'], color='red', s=50, label='40개마다 선택된 데이터')

# 그래프 제목 및 라벨 설정
ax.set_title('CSV 데이터 시각화 (선택된 점 드래그 가능)')
ax.set_xlabel('X 좌표')
ax.set_ylabel('Y 좌표')
ax.legend() # 범례 표시
ax.grid(True) # 그리드 표시
ax.set_aspect('equal', adjustable='box') # X, Y 축 비율을 동일하게 설정

# 드래그 앤 드롭 기능을 위한 변수 초기화
selected_point_index = None # 현재 드래그 중인 점의 인덱스
offset_x, offset_y = 0, 0 # 클릭 지점과 점의 중심 간의 오프셋

def on_press(event):
    """마우스 버튼이 눌렸을 때 호출되는 함수"""
    global selected_point_index, offset_x, offset_y

    if event.inaxes != ax: # 클릭이 현재 axes 내에서 발생했는지 확인
        return

    # 클릭된 점 찾기
    contains, info = draggable_points.contains(event)
    if contains:
        selected_point_index = info['ind'][0] # 클릭된 점의 인덱스
        
        # 클릭 지점과 점의 현재 위치 사이의 오프셋 계산
        current_x, current_y = draggable_points.get_offsets()[selected_point_index]
        offset_x = event.xdata - current_x
        offset_y = event.ydata - current_y

        fig.canvas.mpl_connect('motion_notify_event', on_motion) # 드래그 시작 시 motion 이벤트 연결

def on_motion(event):
    """마우스가 움직일 때 호출되는 함수 (드래그 중)"""
    if selected_point_index is None or event.inaxes != ax:
        return

    if event.xdata is None or event.ydata is None: # 마우스가 그래프 영역을 벗어났을 경우
        return

    # 새로운 위치 계산 (클릭 지점과 점의 오프셋 유지)
    new_x = event.xdata - offset_x
    new_y = event.ydata - offset_y

    # 해당 점의 위치 업데이트
    offsets = draggable_points.get_offsets()
    offsets[selected_point_index] = [new_x, new_y]
    draggable_points.set_offsets(offsets)

    fig.canvas.draw_idle() # 그래프 업데이트

def on_release(event):
    """마우스 버튼이 놓였을 때 호출되는 함수"""
    global selected_point_index
    if selected_point_index is not None:
        selected_point_index = None # 선택된 점 초기화
        fig.canvas.mpl_disconnect('motion_notify_event') # motion 이벤트 연결 해제

# 이벤트 핸들러 연결
fig.canvas.mpl_connect('button_press_event', on_press)
fig.canvas.mpl_connect('button_release_event', on_release)

plt.show()