import json
import keyboard

class NodeDataCollector:
    def __init__(self):
        self.node_data = []
        self.is_collecting = False
        self.file_name = "C:/Users/bovar/OneDrive/바탕 화면/d2/newCatkin_ws/src/br_task/scripts/newProjectD1/Path/node_data.json"
        self.index =0

    def start_collection(self):
        print("데이터 수집을 시작합니다. 수집을 멈추려면 'q' 키를 누르세요.")
        self.is_collecting = True
        self.node_data = []

        keyboard.on_press_key("q", self.stop_collection) #q 누르면 종료

        while self.is_collecting:
            name = input("이름을 입력하세요: ")
            x = float(input("x 좌표를 입력하세요: "))
            y = float(input("y 좌표를 입력하세요: "))
            connected_node = input("연결된 다른 이름을 입력하세요: ")

            data = {} # 데이터 저장할 딕셔너리 선언
            data['Path'] =[] #비어있는 배열 생성
            data['Path'].append({ #json 파일에 들어갈 내용들
                "index" : self.index,
                "name": name,
                "x_coordinate": x,
                "y_coordinate": y,
                "connected name": connected_node
            })
            self.index +=1

            self.node_data.append(data)

    def stop_collection(self, e):
        print("데이터 수집을 종료합니다.")
        self.is_collecting = False
        keyboard.unhook_all()

        with open(self.file_name, "w",encoding="utf-8") as json_file:
            json.dump(self.node_data, json_file, indent=4)

        print(f"{self.file_name} 파일이 생성되었습니다.")


if __name__ == "__main__":
    collector = NodeDataCollector()
    collector.start_collection()
