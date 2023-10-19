#!/usr/bin/env python3

"""
D1 로봇이 정해진 경로를 따라가도록 하는 스크립트
경로 생성 부분
입력 : json파일 (CreatePathJson.py 스크립트 참고)

"""

import json
import math
from CreatePathJson import NodeDataCollector #json 파일을 만드는 파일의 클래스 호출을 위한 선언문


""" Node 선언을 위한 class"""
class Node:
    def __init__(self, index: int, name: str, x: float, y: float):
        # 노드 생성자
        self.index = index  # 노드의 인덱스 정보
        self.name = name    # 노드의 이름
        self.x = x          # X 좌표
        self.y = y          # Y 좌표
        self.next_nodes = []  # 다음 노드를 저장하는 리스트
        self.prev_nodes = []  # 이전 노드를 저장하는 리스트
        self.connected_nodes = self.next_nodes+self.prev_nodes # 연결된 노드와 관계를 저장하는 리스트
        self.path_node = None
        self.g = float('inf')  # 시작 노드에서 현재 노드까지의 실제 비용, 일반적으로 초기화 전에는 무한대로 둠
        self.h = 0  # 현재 노드에서 목적지 노드까지의 휴리스틱 비용
        self.f = float('inf') # 현재 노드의 총 예상 비용

    def add_next_node(self, node):
        # 다음 노드 추가
        self.next_nodes.append(node)

    def add_prev_node(self, node):
        # 이전 노드 추가
        self.prev_nodes.append(node)

    def add_connected_node(self, node):
        """이전 노드 이후 노드 넣기 위함"""
        # 다른 노드와의 연결 정보 추가
        self.add_prev_node(node)  # 다른 노드의 이전 노드로 연결 (문제의 원인)
        node.add_next_node(self)

        # 연결된 노드와의 관계 저장 (relationship은 예를 들면 "Next" 또는 "Prev" 등)
        # self.connected_nodes.append(node)

class CLL:
    def __init__(self):
        self.head = None
        self.start = None
        self.goal = None
        self.save = None # 노드 분기점을 저장하기 위한 변수
        self.nodes = list()  # 노드들의 정보를 가지고 있는 배열
        self.path = None

    def append(self, index: int, name: str, x: float, y: float, distance : int =0 , connected_node: str = None):
        # 노드 추가
        new_node = Node(index, name, x, y, ) #새로운 노드 생성
        self.nodes.append(new_node)
        # print(f"{new_node.name}: {new_node}, 인덱스 : {new_node.index}, 거리 : {new_node.distance}, 연결된 노드 주소 : {connected_node} :")
        if not self.head: #head 노드가 없을 경우
            self.head = new_node #방금 생성된 노드를 헤드에 할당 ( 시작점을 잡음)
        if connected_node:
            connected_node = connected_node.strip()  # 연결된 노드 정보에서 공백 제거
            connected_nodes = connected_node.split(',')  # 연결된 노드 정보를 쉼표로 분리
            for node_name in connected_nodes:
                # 연결된 노드를 찾아서 추가
                # node = self.find_node_by_name(node_name) #이름이 존재하는지 찾은후 있으면 node에 할당 없으면 None 할당
                node = self.fine_node(node_name)  # 이름이 존재하는지 찾은후 있으면 node에 할당 없으면 None 할당
                if node:
                    new_node.add_connected_node(node) # 수정해야함

    def connected_node(self):
        """연결된 노드 자체를 넣는 함수"""
        for i in range(len(self.nodes)):
            self.nodes[i].connected_nodes = self.nodes[i].next_nodes + self.nodes[i].prev_nodes

    def fine_node(self,name):
        for i in range(len(self.nodes)):
            if self.nodes[i].name == name:
                return self.nodes[i]

    def find_path(self,start: int ,end : int,stopover : int =None):
        i = 0
        if not stopover:
            current = self.head
            while current:
                if self.save is not None and len(current.next_nodes) == 0:
                    i = i+1
                    if current.index == start:
                        self.start = current
                    elif current.index == end:
                        self.goal = current
                    try:
                        current = self.save
                        current = current.next_nodes[i]
                    except:
                        break
                if current.index == start:
                    self.start = current
                elif current.index == end:
                    self.goal = current
                if len(current.next_nodes) > 1 and self.save is None:
                    self.save = current
                    current = current.next_nodes[i]
                else:
                    try:
                        current = current.next_nodes[0]
                    except:
                        current = current.next_nodes

        return self.start , self.goal
        # print(f"시작점 : {self.start}")
        # print(f"도착점 : {self.goal}")



class D1Project(CLL): #양방향 연결리스트 생성을 위해 CLL 클래스를 상속받음
    def __init__(self):
        super().__init__()  #CLL 클래스의 생성자를 호출하여 양뱡향 연결 리스트 초기화
        # self.wayPoint()
        self.data = None # Json 파일 내용 저장할 변수
        self.dt = list() # 점과 점 사이거리를 가지고 있는 배열
        '''nodes에 저장된 Node클래스로 생성된 node의 정보에 접근하기 위해서는 
        for i in range self.nodes:
            print(i.name)
            print(i.x_coordinate).... 이런식으로 접근 가능 '''
        self.indexing = dict() # 인덱스와 점의 이름을 매칭할 딕셔너리

    def lodeJsonFile(self): #Json 파일을 읽어오기 위한 함수
        with open("C:/Users/bovar/OneDrive/바탕 화면/d2/newCatkin_ws/src/br_task/scripts/newProjectD1/Path/node_data.json","r") as f: # 경로는 Json 파일 위치에 따라 변경
            self.data = json.load(f) # data[index] 형태로 순차적으로 세부 내용 읽기 가능
        '''
        # name = data[index]['Path'][0]['name'] -> index 값 순번에 위치한 Path을 가르킴
        # ath는 배열 속에 딕셔너리가 존재하기에 0번째 고정 그 후 딕셔너리 키 값으로 값 추출
        '''
        for i in range(len(self.data)):
            self.indexing[i] = self.data[i]['Path'][0]['name']

    def cll(self):
        for i in range(len(self.data)):
            super().append(self.data[i]['Path'][0]['index'],
                           self.data[i]['Path'][0]['name'],
                           self.data[i]['Path'][0]['x_coordinate'],
                           self.data[i]['Path'][0]['y_coordinate'],
                           i,
                           self.data[i]['Path'][0]['connected name']
                           )

    def distance(self):
        for i in range(len(self.data)-1):
            x2, x1 = self.data[i+1]['Path'][0]['x_coordinate'], self.data[i]['Path'][0]['x_coordinate']
            y2, y1 = self.data[i+1]['Path'][0]['y_coordinate'], self.data[i]['Path'][0]['y_coordinate']
            self.dt.append(math.sqrt((x2-x1)**2+(y2-y1)**2))

    def find_path2(self, start_index: int, end_index: int):
        # 시작 노드와 목표 노드를 이름으로 찾습니다.
        start_node,end_node = self.find_path(start_index,end_index)
        # print(f"시작 노드 : {start_node}, 목표노드 : {end_node}")

        if not start_node or not end_node:
            print("시작 노드 또는 목표 노드를 찾을 수 없습니다.")
            return

            # A* 알고리즘을 위한 초기화
        open_list = [start_node]  # 시작 노드에서 출발하는 경로의 open 리스트
        path_list = [start_node]
        # 시작 노드의 g, h, f 값을 초기화합니다.
        start_node.g = 0
        start_node.h = self.calculate_heuristic(start_node, end_node)  # H의 값은 장애물을 고려하지 않은 시작점에서 목표점까지 거리
        start_node.f = start_node.h
        now_node = start_node
        while open_list:
            # 현재 노드는 f의 값이 가장 적은 것 선택합니다.
            current_node = self.get_node_with_lowest_f(open_list,end_node)
            now_node.path_node =current_node
            now_node = current_node
            open_list.remove(current_node)

            if current_node not in path_list:
                path_list.append(current_node)
            elif current_node not in path_list and open_list:
                current_node = self.get_node_with_lowest_f(open_list, end_node)
                open_list.remove(current_node)

            # 목표 노드에 도달한 경우 최단 경로를 찾았습니다.
            if current_node == end_node:
                self.path = self.construct_path(start_node)
                return

            # 현재 노드의 이웃 노드를 탐색합니다.
            neighbors = current_node.connected_nodes
            for neighbor in neighbors:
                if neighbor not in open_list:
                    open_list.append(neighbor)
                if neighbor in path_list:
                    open_list.remove(neighbor)

                tentative_g = current_node.g + self.calculate_distance(current_node, neighbor)

                if tentative_g < neighbor.g:
                    neighbor.g = tentative_g
                    neighbor.f = neighbor.g + self.calculate_heuristic(neighbor, end_node)

        print("최단 경로를 찾을 수 없습니다.")

    def calculate_distance(self, node1, node2):
        # 두 노드 사이의 거리 계산
        x1, y1 = node1.x, node1.y
        x2, y2 = node2.x, node2.y
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def calculate_heuristic(self, node, goal_node):
        # 휴리스틱 함수 계산 (예: 유클리디안 거리)
        x1, y1 = node.x, node.y
        x2, y2 = goal_node.x, goal_node.y
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_node_with_lowest_f(self, node_list,end_node):
        # f 값이 가장 낮은 노드 반환
        return min(node_list, key=lambda node: node.f)

    def construct_path(self,start_node): # 최단 경로 생성
        path = []
        current_node = start_node
        path.insert(0, current_node)
        while current_node.path_node is not None:
            path.append(current_node.path_node)
            current_node = current_node.path_node
        print("최단 경로:", " -> ",path)
        return path

    def execute(self, start, end):
        self.lodeJsonFile()
        self.cll()
        self.connected_node()
        self.find_path2(start, end)
        return self.path


def main():
    d1 = D1Project()
    d1.lodeJsonFile()
    d1.cll()
    d1.connected_node()
    d1.find_path2(0,3)
    '''
    #Json 경로를 만들기 위한 main에 실행되어야할 함수 명령어
    JsonCreate = NodeDataCollector() # JsonCreate에 NodeDataCollector 클래스 할당
    JsonCreate.start_collection() # NodeDataCollector 클래스 내부 함수 호출
    '''


if __name__ =='__main__':
    main()