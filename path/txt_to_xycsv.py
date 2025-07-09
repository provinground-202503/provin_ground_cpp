import re

def convert_text_file(input_filepath, output_filepath):
    """
    주어진 텍스트 파일을 X,Y 형식의 CSV 파일로 변환합니다.
    여러 개의 공백을 하나의 쉼표로 정확히 변환합니다.

    Args:
        input_filepath (str): 원본 텍스트 파일의 경로.
        output_filepath (str): 변환된 CSV 파일이 저장될 경로.
    """
    try:
        with open(input_filepath, 'r') as infile:
            lines = infile.readlines()

        with open(output_filepath, 'w') as outfile:
            outfile.write("X,Y\n")  # 헤더 작성
            for line in lines:
                # 각 줄의 앞뒤 공백 제거
                stripped_line = line.strip()
                
                # 하나 이상의 공백을 쉼표로 대체
                # re.sub(r'\s+', ',', stripped_line) 는 정규식을 사용하여
                # 하나 이상의 공백(\s+)을 쉼표(,)로 대체합니다.
                converted_line = re.sub(r'\s+', ',', stripped_line)
                
                outfile.write(converted_line + "\n")
        print(f"파일 변환이 성공적으로 완료되었습니다: {output_filepath}")
    except FileNotFoundError:
        print(f"오류: 파일을 찾을 수 없습니다. 경로를 확인해주세요: {input_filepath}")
    except Exception as e:
        print(f"파일 변환 중 오류가 발생했습니다: {e}")

# --- 사용 예시 ---
# 변환할 원본 파일의 경로
input_file = "interpolated_ctrack_vils.txt" # 실제 파일 경로로 변경해주세요.

# 변환된 파일이 저장될 경로
output_file = "interpolated_ctrack_vils.csv" # 원하는 출력 파일명으로 변경 가능합니다.

# 예시 input.txt 파일 생성 (테스트용)
# 실제 사용 시 이 부분은 제거하거나 주석 처리하세요.
# with open(input_file, 'w') as f:
#     f.write("360950.688405433    4065819.88165149\n")
#     f.write("360950.69682115293  4065819.581777824\n")
#     f.write("360950.70007010695  4065819.281805404\n")
#     f.write("360950.69928891165  4065818.981774898\n")
#     f.write("360950.6960922506   4065818.6817372353\n")

# 함수 호출
convert_text_file(input_file, output_file)