import pandas as pd

# 1. CSV 파일 불러오기
try:
    df = pd.read_csv('pathmaker_path_xy_2.csv')
    print("원본 데이터의 일부:")
    print(df.head())
    print("\n데이터 정보:")
    df.info()
except FileNotFoundError:
    print("Error: 'path.csv' 파일을 찾을 수 없습니다. 파일 경로를 확인해주세요.")
    exit()

# 2. 문제 구간의 인덱스 또는 조건 지정
# 예시: 인덱스 2번부터 5번(5번 포함)까지의 구간을 보정
start_index = 300
end_index = 330

# 또는 특정 x, y 값 범위를 기준으로 구간을 지정할 수도 있습니다.
# 예를 들어, x가 12.0보다 크고 15.0보다 작은 모든 지점
# problematic_segment = df[(df['x'] > 12.0) & (df['x'] < 15.0)]

# 3. 보정량 결정 (예시: x 좌표를 왼쪽으로 0.5만큼 이동)
correction_amount_x = 0.2 # 왼쪽으로 이동하므로 음수 값

# 4. 해당 구간의 x 좌표 보정 적용
# .loc을 사용하여 특정 구간의 데이터를 안전하게 수정합니다.
df.loc[start_index:end_index, 'Y'] = df.loc[start_index:end_index, 'Y'] + correction_amount_x

print(f"\n인덱스 {start_index}부터 {end_index}까지 x 좌표를 {correction_amount_x}만큼 보정했습니다.")
print("수정된 데이터의 일부 (보정된 구간 포함):")
print(df.iloc[start_index-2:end_index+2]) # 보정 전후를 확인하기 위해 넓게 출력

# 5. 수정된 데이터를 새로운 CSV 파일로 저장 (원본 파일 백업 권장)
output_filename = 'pathmaker_path_xy_3.csv'
df.to_csv(output_filename, index=False)
print(f"\n수정된 경로 데이터가 '{output_filename}'으로 저장되었습니다.")

# 6. (선택 사항) y 좌표도 보정해야 하는 경우
# correction_amount_y = 0.2 # 예를 들어 y 좌표를 위로 0.2만큼 이동
# df.loc[start_index:end_index, 'y'] = df.loc[start_index:end_index, 'y'] + correction_amount_y
# df.to_csv(output_filename, index=False)
# print(f"\ny 좌표도 보정되어 '{output_filename}'으로 저장되었습니다.")