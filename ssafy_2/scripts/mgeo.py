#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

# mgeo 는 정밀도로지도 데이터 인 Mgeo(MORAI Geometry) 데이터를 읽어오는 예제입니다.
# Json 파일 형식으로 되어 있는 Mgeo 데이터를 dictionary 형태로 읽어옵니다.

# 노드 실행 순서 
# 1. Mgeo data 읽어온 후 데이터 확인

#TODO: (1) Mgeo data 읽어온 후 데이터 확인
load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
mgeo_planner_map = MGeo.create_instance_from_json(load_path)

node_set = mgeo_planner_map.node_set
link_set = mgeo_planner_map.link_set
nodes=node_set.nodes
links=link_set.lines

print('# of nodes: ', len(node_set.nodes))
print('# of links: ', len(link_set.lines))