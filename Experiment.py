import random
import numpy
import math
import time
import collections
# random.seed(10)
import gurobipy as gp
from gurobipy import GRB
import json
from email.mime.text import MIMEText
import smtplib
#from func_timeout import func_timeout, FunctionTimedOut, func_set_timeout
import threading
#useNetworkX
import networkx as nx

def GenerateMap(Mapsize, Obs_ratio, Robot_num):
    Arr = numpy.zeros((Mapsize, Mapsize))
    ran_list = list(range(0, Mapsize * Mapsize))
    Obs_num = int((Mapsize * Mapsize) * Obs_ratio)
    Robot_position = int(Robot_num * 2)

    output_list = random.sample(ran_list, Obs_num + Robot_position)

    Obs_list = output_list[0:Obs_num]
    Robot_list = output_list[Obs_num:]

    for i in Obs_list:
        row = int(i / Mapsize)
        col = i % Mapsize
        Arr[row][col] = 1

    s = 2
    for j in range(0, len(Robot_list)):
        if (j % 2 == 0):
            # start
            i = Robot_list[j]
            row = int(i / Mapsize)
            col = i % Mapsize
            Arr[row][col] = -1 * s
        else:
            i = Robot_list[j]
            row = int(i / Mapsize)
            col = i % Mapsize
            Arr[row][col] = s
            s = s + 1

    return Arr.astype(int)
def timespace_map_ILP(MapArr, t_ratio):
    # getAdj
    D1_map = MapArr.reshape(-1)
    D1_map_node = []
    Adj = [[] for i in range((len(MapArr) * len(MapArr)))]
    map_len = len(MapArr)

    for x in range(0, len(D1_map)):
        if (D1_map[x] != 1):
            D1_map_node.append(x)

    for i in range(0, len(MapArr)):
        for j in range(0, len(MapArr)):
            if (MapArr[i][j] != 1):

                try:
                    if (MapArr[i][j + 1] != 1):
                        Adj[(i * map_len + j)].append(i * map_len + j + 1)
                except:
                    pass

                try:
                    if (MapArr[i][j - 1] != 1 and (j - 1) >= 0):
                        Adj[(i * map_len + j)].append(i * map_len + j - 1)
                except:
                    pass

                try:
                    if (MapArr[i - 1][j] != 1 and (i - 1) >= 0):
                        Adj[(i * map_len + j)].append((i - 1) * map_len + j)
                except:
                    pass

                try:
                    if (MapArr[i + 1][j] != 1):
                        Adj[(i * map_len + j)].append((i + 1) * map_len + j)
                except:
                    pass

    conv_dict = {}
    for i in range(0, len(D1_map_node)):
        # print(i,D1_map_node[i])
        conv_dict[D1_map_node[i]] = i

    # CreateMap
    # add Basic Nodes
    DiG_root = nx.DiGraph()
    map_len = len(D1_map_node)
    prime = 0
    for i in range(0, len(MapArr) * t_ratio * 2 + 1):
        prime = prime + 1
        if (i != 0):
            if (prime % 2 == 1):
                for j in range(0, map_len):
                    tempAdj = [conv_dict[x] + (map_len) * i for x in Adj[D1_map_node[j]]]
                    DiG_root.add_node(j + (map_len) * i, Gadget=False, Attr=D1_map[D1_map_node[j]], Adja=tempAdj,
                                      Prime=False)
            else:
                for j in range(0, map_len):
                    tempAdj = [(j + (map_len) * i) + map_len]
                    DiG_root.add_node(j + (map_len) * i, Gadget=False, Attr=D1_map[D1_map_node[j]], Adja=tempAdj,
                                      Prime=True)
        else:
            for j in range(0, map_len):
                tempAdj = [conv_dict[x] + (map_len) * i for x in Adj[D1_map_node[j]]]
                DiG_root.add_node(j + (map_len) * i, Gadget=False, Attr=D1_map[D1_map_node[j]], Adja=tempAdj,
                                  Prime=False)

    #     #Add Basic Edges
    Prime = nx.get_node_attributes(DiG_root, 'Prime')
    for i in range(0, len(MapArr) * t_ratio * 2):
        for j in range(0, map_len):
            if (Prime[j + (map_len) * i] == True):
                DiG_root.add_edge(j + (map_len) * i, j + (map_len) * (i + 1), capa=1, cost=0)
            else:
                DiG_root.add_edge(j + (map_len) * i, j + (map_len) * (i + 1), capa=1, cost=1)

    # Gadget
    G = DiG_root.copy()
    temp_G = DiG_root.copy()

    Gadget_start_index = len(G.nodes)
    layer_num = len(MapArr) * t_ratio * 2 + 1
    num_in_onelayer = len(D1_map_node)
    final_layer_index = num_in_onelayer * (layer_num - 1)

    for (p, d) in temp_G.nodes(data=True):
        if (p < final_layer_index):
            if (d['Gadget'] == False):
                if (d['Prime'] == False):
                    for x in d['Adja']:
                        for (k, v) in G.nodes(data=True):
                            if (v['Gadget'] == True):
                                if ((v['pos_a'] == p and v['pos_b'] == x)):
                                    find_res = k
                                    break
                                elif ((v['pos_a'] == x and v['pos_b'] == p)):
                                    find_res = k
                                    break
                                else:
                                    find_res = False
                            else:
                                find_res = False

                        if (find_res != False):
                            G.add_edge(p, find_res, capa=1, cost=0)
                        else:
                            G.add_node(Gadget_start_index, Gadget=True, pos_a=p, pos_b=x)
                            G.add_edge(p, Gadget_start_index, capa=1, cost=0)
                            Gadget_start_index = Gadget_start_index + 1

    new_index = len(G.nodes)

    temp_G = G.copy()
    for (p, d) in temp_G.nodes(data=True):
        if (d['Gadget'] == True):
            G.add_node(new_index, Gadget=True, pos_c=d['pos_a'] + map_len, pos_d=d['pos_b'] + map_len)
            G.add_edge(p, new_index, capa=1, cost=1)
            G.add_edges_from([(new_index, d['pos_a'] + map_len), (new_index, d['pos_b'] + map_len)], capa=1, cost=0)
            new_index = new_index + 1

    start_dict = {}
    goal_dict = {}

    for (p, d) in G.nodes(data=True):
        if (d['Gadget'] == False):
            if (p < map_len):
                if (d['Attr'] < 0):
                    start_dict[abs(d['Attr'])] = p
            if (p >= final_layer_index):
                if (d['Attr'] > 1):
                    goal_dict[abs(d['Attr'])] = p

    # set Loopback edges

    for (p, d) in goal_dict.items():
        G.add_edge(d, start_dict[p], capa=1, cost=0, loopback=True)

    return [G, start_dict, goal_dict]
def timespace_map_Q(MapArr, t_ratio):
    D1_map = MapArr.reshape(-1)
    nodenum_onelayer = len(D1_map)
    Map_length = len(MapArr)

    # print(MapArr)

    # getAdj

    Adj = [[] for i in range((Map_length * Map_length))]

    for i in range(0, Map_length):
        for j in range(0, Map_length):

            if (MapArr[i][j] != 1):
                try:
                    if (MapArr[i][j + 1] != 1):
                        Adj[(i * Map_length + j)].append(i * Map_length + j + 1)
                except:
                    pass

                try:
                    if (MapArr[i][j - 1] != 1 and (j - 1) >= 0):
                        Adj[(i * Map_length + j)].append(i * Map_length + j - 1)
                except:
                    pass

                try:
                    if (MapArr[i - 1][j] != 1 and (i - 1) >= 0):
                        Adj[(i * Map_length + j)].append((i - 1) * Map_length + j)
                except:
                    pass

                try:
                    if (MapArr[i + 1][j] != 1):
                        Adj[(i * Map_length + j)].append((i + 1) * Map_length + j)
                except:
                    pass

                    # CreateMap
    G = nx.DiGraph()
    mapping = {}
    # add Basic node
    for i in range(0, Map_length * t_ratio * 2 + 1):
        if i != (Map_length * t_ratio * 2):
            # except final

            if i % 2 == 0:
                # Adj gadget
                for j in range(0, nodenum_onelayer):
                    tempAdj = [x + nodenum_onelayer * i for x in Adj[j]]
                    G.add_node(j + nodenum_onelayer * i, Gadget=False, Attr=D1_map[j], Adja=tempAdj, Prime=True)
                    mapping[(j + nodenum_onelayer * i)] = "t" + str(int(i / 2)) + "x" + str(j % Map_length) + "y" + str(
                        int(j / Map_length))
            else:
                # Adj self
                for j in range(0, nodenum_onelayer):
                    if (D1_map[j] == 1):
                        tempAdj = []
                    else:
                        tempAdj = [j + nodenum_onelayer * (i + 1)]
                    G.add_node(j + nodenum_onelayer * i, Gadget=False, Attr=D1_map[j], Adja=tempAdj, Prime=False)

        else:
            for j in range(0, nodenum_onelayer):
                tempAdj = [j + nodenum_onelayer * (i + 1)]
                G.add_node(j + nodenum_onelayer * i, Gadget=False, Attr=D1_map[j], Adja=[], Prime=None)
                mapping[(j + nodenum_onelayer * i)] = "t" + str(int(i / 2)) + "x" + str(j % Map_length) + "y" + str(
                    int(j / Map_length))
    # add Basic edge
    Prime = nx.get_node_attributes(G, 'Prime')
    Adja = nx.get_node_attributes(G, 'Adja')
    for i in range(0, Map_length * t_ratio * 2):
        for j in range(0, nodenum_onelayer):
            if (len(Adja[(nodenum_onelayer * i) + j]) > 0):
                if (Prime[(nodenum_onelayer * i) + j] == True):
                    G.add_edge((nodenum_onelayer * i) + j, (nodenum_onelayer * (i + 1)) + j, capa=1, cost=1)
                else:
                    G.add_edge((nodenum_onelayer * i) + j, (nodenum_onelayer * (i + 1)) + j, capa=1, cost=0)

    # Gadget
    temp_G = G.copy()
    Gadget_start_index = len(G.nodes)
    Gadget_mapping = {}

    for (p, d) in temp_G.nodes(data=True):
        if (d['Prime'] != None):
            if (len(d['Adja']) > 0):
                if (d['Gadget'] == False):
                    if (d['Prime'] == True):
                        for a in d['Adja']:
                            for (k, v) in G.nodes.data():
                                find = False
                                if (v['Gadget'] == True):
                                    if (v['Adja'] == [p, a] or v['Adja'] == [a, p]):
                                        G.add_edge(p, k, capa=1, cost=0)
                                        find = True
                                        Gadget_mapping[(p,a+nodenum_onelayer)] = k
                                        break
                            if (find == False):
                                G.add_node(Gadget_start_index, Gadget=True, Adja=[p, a])
                                G.add_edge(p, Gadget_start_index, capa=1, cost=0)
                                Gadget_mapping[(p,a+nodenum_onelayer)] = Gadget_start_index
                                Gadget_start_index = Gadget_start_index + 1

    temp_G = G.copy()
    for (p, d) in temp_G.nodes(data=True):
        if (d['Gadget'] == True):
            tempAdja = [x + nodenum_onelayer for x in d['Adja']]
            G.add_node(Gadget_start_index, Gadget=True, Adja=tempAdja)
            G.add_edge(p, Gadget_start_index, capa=1, cost=1)
            for i in tempAdja:
                G.add_edge(Gadget_start_index, i, capa=1, cost=0)

            Gadget_start_index = Gadget_start_index + 1

    start_dict = {}
    goal_dict = {}

    for (p, d) in G.nodes.data():
        if (d['Gadget'] == False):
            if (p < nodenum_onelayer):
                if (d['Attr'] < 0):
                    start_dict[abs(d['Attr'])] = p
            if (d['Prime'] == None):
                if (d['Attr'] > 1):
                    goal_dict[abs(d['Attr'])] = p

    return [G, start_dict, goal_dict, mapping , Gadget_mapping]
def timespace_map(MapArr, t_ratio):
    D1_map = MapArr.reshape(-1)
    nodenum_onelayer = len(D1_map)
    Map_length = len(MapArr)

    # getAdj
    Adj = [[] for i in range((Map_length * Map_length))]
    for i in range(0, Map_length):
        for j in range(0, Map_length):

            if (MapArr[i][j] != 1):
                try:
                    if (MapArr[i][j + 1] != 1):
                        Adj[(i * Map_length + j)].append(i * Map_length + j + 1)
                except:
                    pass

                try:
                    if (MapArr[i][j - 1] != 1 and (j - 1) >= 0):
                        Adj[(i * Map_length + j)].append(i * Map_length + j - 1)
                except:
                    pass

                try:
                    if (MapArr[i - 1][j] != 1 and (i - 1) >= 0):
                        Adj[(i * Map_length + j)].append((i - 1) * Map_length + j)
                except:
                    pass

                try:
                    if (MapArr[i + 1][j] != 1):
                        Adj[(i * Map_length + j)].append((i + 1) * Map_length + j)
                except:
                    pass

    # add Basic node
    G = nx.DiGraph()
    mapping = {}

    for i in range(0, Map_length * t_ratio + 1):
        if (i != Map_length * t_ratio):
            for j in range(0, nodenum_onelayer):
                tempAdj = [x + nodenum_onelayer * (i + 1) for x in Adj[j]]
                tempAdj.append(j + nodenum_onelayer * (i + 1))
                G.add_node(j + nodenum_onelayer * i, Attr=D1_map[j], adja=tempAdj)
                mapping[(j + nodenum_onelayer * i)] = "t" + str(i) + "x" + str(j % Map_length) + "y" + str(
                    int(j / Map_length))
        else:
            for j in range(0, nodenum_onelayer):
                tempAdj = []
                G.add_node(j + nodenum_onelayer * i, Attr=D1_map[j], adja=tempAdj)
                mapping[(j + nodenum_onelayer * i)] = "t" + str(i) + "x" + str(j % Map_length) + "y" + str(
                    int(j / Map_length))
    final_index = (Map_length * t_ratio) * nodenum_onelayer - 1

    # add edge
    # Adja = nx.get_node_attributes(G, 'Adja')
    for k, v in G.nodes(data=True):
        G.add_edges_from([(k, x) for x in v['adja']])

    start_dict = {}
    goal_dict = {}

    for (p, d) in G.nodes(data=True):
        if (p < nodenum_onelayer):
            if (d['Attr'] < 0):
                start_dict[abs(d['Attr'])] = p
        if (p > final_index):
            if (d['Attr'] > 1):
                goal_dict[abs(d['Attr'])] = p

    #     G = nx.relabel_nodes(G,mapping,copy = False)

    return [G, start_dict, goal_dict, mapping]
def CalMultiPathCost(G, path_dict):
    # Hyperpara penalty
    penalty = 9999
    path_list = list(path_dict.values())
    # update edge capacity
    temp_G = G.copy()

    for p in path_list:
        capa = nx.get_edge_attributes(G, 'capa')
        for i in range(0, len(p) - 1):
            temp_capa = capa[p[i], p[i + 1]]
            temp_attr = {(p[i], p[i + 1]): {'capa': temp_capa - 1}}
            nx.set_edge_attributes(temp_G, temp_attr)

    capa_2 = nx.get_edge_attributes(temp_G, 'capa')
    Gadget = nx.get_node_attributes(temp_G, 'Gadget')
    Attr = nx.get_node_attributes(temp_G, 'Attr')
    cost_att = nx.get_edge_attributes(temp_G, 'cost')

    path_cost = {}
    for k, p in path_dict.items():
        cost = 0
        for i in range(1, len(p)):
            if (capa_2[p[i - 1], p[i]] < 0):
                cost = cost + penalty

            elif ((Gadget[p[i - 1]] == False) and (Gadget[p[i]] == False)):
                if (Attr[p[i - 1]] == Attr[p[-1]] and Attr[p[i - 1]] == Attr[p[i]]):
                    cost = cost + 0
                else:
                    cost = cost + cost_att[p[i - 1], p[i]]
            else:
                cost = cost + cost_att[p[i - 1], p[i]]

        path_cost[k] = cost

    return path_cost
def CalSinglePathCost(G, path):
    # wait at end cost 0
    cost = 0
    cost_att = nx.get_edge_attributes(G, 'cost')
    Gadget = nx.get_node_attributes(G, 'Gadget')
    Attr = nx.get_node_attributes(G, 'Attr')

    for i in range(1, len(path)):
        if ((Gadget[path[i - 1]] == False) and (Gadget[path[i]] == False)):
            if (Attr[path[i - 1]] == Attr[path[-1]] and Attr[path[i - 1]] == Attr[path[i]]):
                cost = cost + 0
            else:
                cost = cost + cost_att[path[i - 1], path[i]]
        else:
            cost = cost + cost_att[path[i - 1], path[i]]

    return cost
def GetAllKeyByValue(dic, search):
    return [k for k, v in dic.items() if v == search]
def path_mapping(path , timemap, nodenum_onelayer, ori_map_mapping, time_map_mapping, Gadget_mapping):
    p = []
    time_map_mapping_inv = {v: k for k, v in time_map_mapping.items()}

    for i in range(0, len(path)):
        if (i != len(path) - 1):
            mapping_l = ori_map_mapping[path[i]]
            mapping_r = ori_map_mapping[path[i + 1]]
            time_mapping_l =  time_map_mapping_inv[mapping_l]
            above_time_mapping_r = (time_map_mapping_inv[mapping_r]-nodenum_onelayer)

            # checkifStay
            if(mapping_l.split('x')[1] == mapping_r.split('x')[1]):
                temp_p = ([time_mapping_l,above_time_mapping_r])
            else:
                Gad = Gadget_mapping[(time_mapping_l,above_time_mapping_r)]
                Gad_r = list(timemap[Gad])[0]
                temp_p = ([time_mapping_l,Gad,Gad_r,above_time_mapping_r])

            # temp_p = list(all_simple_paths_graph_modify(timemap, time_mapping_l, above_time_mapping_r,nodenum_onelayer))

            p.extend(temp_p)

    #add goal vertex
    p.append(time_map_mapping_inv[mapping_r])
    return p
def all_simple_paths_graph(G, source, targets):
    visited = collections.OrderedDict.fromkeys([source])
    stack = [iter(G[source])]

    while stack:
        children = stack[-1]
        child = next(children, None)
        if child is None:
            stack.pop()
            visited.popitem()
        elif len(visited):
            if child in visited:
                continue

            if child == targets:
                yield list(visited) + [child]
            else:
                visited[child] = None
                stack.append(iter(G[child]))
def all_simple_paths_graph_modify(G, source, targets, nodenum_onelayer):
    Gadget = nx.get_node_attributes(G, 'Gadget')
    Attr = nx.get_node_attributes(G, 'Attr')

    visited = collections.OrderedDict.fromkeys([source])
    stack = [iter(G[source])]

    while stack:
        children = stack[-1]
        child = next(children, None)
        if child is None:
            stack.pop()
            visited.popitem()
        elif len(visited):
            if child in visited:
                continue

            if (len(visited) >= 3 and Gadget[child] == False):
                rev_list = list(reversed(visited))
                if (Gadget[rev_list[0]] == True):
                    if (child == rev_list[2] + nodenum_onelayer):
                        continue
            if child == targets:
                yield list(visited) + [child]
            else:
                visited[child] = None
                stack.append(iter(G[child]))
def get_path_dict(map, timemap, start_dict, goal_dict, map_mapping, time_map_mapping, Gadget_mapping, nodenum_onelayer):
    all_path_dict = {}
    #start,goal_dict is generate from map
    for k,v in start_dict.items():
        mapping_path = []
        temp_p = list(all_simple_paths_graph(map,v,goal_dict[k]))
        for i in temp_p:
            mapping_path.append(path_mapping(i,timemap,nodenum_onelayer, map_mapping, time_map_mapping, Gadget_mapping))
        all_path_dict[k] = mapping_path

    return all_path_dict

def ILP(G, start_dict, goal_dict):
    # Gurobi preprocessing
    nodes = list(G.nodes)
    C1 = nx.get_edge_attributes(G, 'capa')
    C2 = nx.get_edge_attributes(G, 'cost')
    e = []

    loopback = nx.get_edge_attributes(G, 'loopback')
    edges = list(G.edges)
    for i in range(1, len(start_dict) + 1):
        for j in loopback:
            if (j[1] == start_dict[i + 1]):
                e.append(j)
                edges.remove(j)
    e.extend(edges)

    Robot = []
    for i in range(1, len(start_dict) + 1):
        Robot.append(i)

    Robot_num = len(Robot)

    e_index = []
    index_to_edge = {}

    for i in range(len(e)):
        e_index.append(i + 1)
        index_to_edge[i + 1] = e[i]

    inverse_index_to_edge = dict((v, k) for k, v in index_to_edge.items())

    # Gurobi
    m = gp.Model('netflow')
    m.setParam('OutputFlag',False)
    # Create variables
    x = m.addVars(Robot, e_index, name="x")

    m.setObjective(gp.quicksum(x[i, i] for i in range(1, Robot_num + 1)), GRB.MAXIMIZE)
    # Constraint1
    m.addConstrs(
        (x.sum('*', i) <= 1 for i in e_index), "cons1")

    # Constraint2
    m.addConstrs(
        (x[i, j] == 0 for i in range(1, Robot_num + 1)
         for j in range(1, Robot_num + 1)
         if i != j), name='Cons2')

    # Constraint3
    for v in nodes:
        for r in range(1, Robot_num + 1):
            temp_in_edges = []
            temp_out_edges = []
            for i in G.in_edges(v):
                temp_in_edges.append(inverse_index_to_edge[i])
            for i in G.out_edges(v):
                temp_out_edges.append(inverse_index_to_edge[i])

            m.addConstr(
                (gp.quicksum(x[r, j] for j in temp_in_edges) ==
                 gp.quicksum(x[r, j] for j in temp_out_edges)
                 ))

    m.optimize()

    #     all_path = []
    all_path_dict = {}
    if m.status == GRB.OPTIMAL:
        solution = m.getAttr('x', x)
        for r in Robot:
            #             print('\n',r)
            all_path_dict[r + 1] = []
            for j in e_index:
                if solution[r, j] > 0:
                    #                     print(r,j,index_to_edge[j])
                    #                     all_path.append(index_to_edge[j])
                    all_path_dict[r + 1].append(index_to_edge[j])

    Robot_path = {}
    for (p, d) in start_dict.items():
        Robot_path[p] = [d]
        while ((Robot_path[p])[-1] != goal_dict[p]):
            if (len(all_path_dict[p]) < 1):
                print('error,ILP go next round')
                break

            temp_list = all_path_dict[p].copy()
            for i in temp_list:
                if (i[0] == (Robot_path[p])[-1]):
                    Robot_path[p].append(i[1])
                    all_path_dict[p].remove(i)
                    if ((Robot_path[p])[-1] == goal_dict[p]):
                        break

    path_cost = CalMultiPathCost(G, Robot_path)
    makespan = max(path_cost.values())


    obj = m.getObjective()

    obj_getValue = obj.getValue()

    return [makespan, obj_getValue]

def Main_ILP(Grid_size,Obs_ratio,Robot_num,Arr):
    t = time.time()
    for i in range(1,3):
        #print('T-ratio:',i)
        ILP_G, ILP_start, ILP_goal = timespace_map_ILP(Arr,i)
        Makespan_ILP, ObjValue = ILP(ILP_G,ILP_start,ILP_goal)
        if(ObjValue == Robot_num):
            print('ILP find sol')
            return [Makespan_ILP,(time.time()-t)]
        else:
            continue

    #if T = 2 can't find
    print('ILP cannot find in this round')
    return False

def QLearning(G, start_dict, goal_dict, nodenum_onelayer, T, L, E, all_path_dict):
    # L(Lambda) E(Epilson)
    # tStart = time.time()
    Convergence_count = T * 0.2

    # get valid path (del through gadget stay)
    Valid_all_path_dict = all_path_dict.copy()

    Q_table = {}
    Q_table_count = {}
    for k, v in Valid_all_path_dict.items():
        Q_table[k] = {i: CalSinglePathCost(G, v[i]) for i in range(0, len(v))}
        Q_table_count[k] = {i: 0 for i in range(0, len(v))}

    action_list = {}
    action_index_list = {}


    for i in range(0, T):
        if (Convergence_count <= 0):
            #             print('Convergence')
            break
        a = math.pow(L, i + 1)
        b = math.pow(E, i + 1)  # epsilon

        #         print('T:',i)
        #         print('a:',a)
        #         print('epsilon:',b)

        for k, v in Valid_all_path_dict.items():
            # epsilon-greedy
            if (len(v) == 0):
                #print('agent', k, 'cannot get valid path')
                return

            temp_float = random.random()

            if (temp_float <= b):
                # random
                temp_rand = random.randint(0, len(v) - 1)
                action = v[temp_rand]
                action_index_list[k] = temp_rand

            else:
                # greedy

                best = min(list(Q_table[k].values()))
                best_key_list = GetAllKeyByValue(Q_table[k], best)
                temp_rand = random.randint(0, len(best_key_list) - 1)

                action = v[best_key_list[temp_rand]]
                action_index_list[k] = best_key_list[temp_rand]

            action_list[k] = (action)

        # cal travel time
        path_cost = CalMultiPathCost(G, action_list)

        for k, v in Q_table.items():
            action_index = action_index_list[k]
            #             print('robot',k,'update action',action_index,':')
            #             print(Q_table[k][action_index],'--------->',(1-a)*Q_table[k][action_index] + (a * path_cost[k]))
            if (Q_table[k][action_index] == ((1 - a) * Q_table[k][action_index] + (a * path_cost[k]))):
                Convergence_count = Convergence_count - 1
            else:
                Convergence_count = T * 0.2

            Q_table[k][action_index] = (1 - a) * Q_table[k][action_index] + (a * path_cost[k])
            Q_table_count[k][action_index] = Q_table_count[k][action_index] + 1

    #         print('---------------------------------------------')
    # tEnd = time.time()

    # get best action
    for k, v in Valid_all_path_dict.items():
        best = min(list(Q_table[k].values()))

        best_key_list = GetAllKeyByValue(Q_table[k], best)
        count = [Q_table_count[k][i] for i in best_key_list]
        count_most = numpy.argmax(count)

        action = v[best_key_list[count_most]]
        action_index_list[k] = best_key_list[count_most]
        action_list[k] = action

    #         print('select action:', action_index_list[k])

    path_cost = CalMultiPathCost(G, action_list)
    makespan = max(path_cost.values())

    #     All_time_End = time.time()
    #     All_time = All_time_End - All_time_start
    # Q_Time = tEnd - tStart

    return makespan
def Main_Q(Grid_size,Obs_ratio,Robot_num,Arr):
    D1_map = Arr.reshape(-1)
    nodenum_onelayer = len(D1_map)
    Q_time_list = []
    Part_Q_time_list = []
    Q_res = []
    Part_Q_res = []
    Q_suc = False
    Part_Q_suc = False

    for i in range(1,3):
        path_find = True
        map_time = time.time()
        timespace_G, timespace_s, timespace_goal, timespace_G_mapping, Gadget_mapping = timespace_map_Q(Arr, i)
        Q_time_list.append(time.time() - map_time)
        Part_Q_time_list.append(time.time() - map_time)
        G, s, goal, G_mapping = timespace_map(Arr, i)
        path_dict = get_path_dict(G, timespace_G, s, goal, G_mapping, timespace_G_mapping, Gadget_mapping,nodenum_onelayer)
        for (k,v) in path_dict.items():
            if(len(v)==0):
                path_find = False
        if(path_find==False):
            print('path cannot find in Q next round')
            continue
        print('Q Path ok')
        #Q-Learning
        try:
            if(Q_suc == False):
                Q_time_start = time.time()
                makespan_Q = QLearning(timespace_G, timespace_s, timespace_goal, nodenum_onelayer, 100, 0.9, 0.1,path_dict)
                Q_time_end = time.time()
                Q_time_list.append(Q_time_end-Q_time_start)
                if(makespan_Q<1000):
                    Q_res.extend([makespan_Q, sum(Q_time_list)])
                    Q_suc = True
                    print('Full Q suc')
        except:
            print('All-Q-Learning Fail')

        try:
            if(Part_Q_suc==False):
                for j in range(0,5):

                    temp_path_dict = {}
                    for k in path_dict.keys():
                        sample_num = int(math.sqrt(len(path_dict[k])))
                        temp_path_dict[k] = random.sample(path_dict[k],sample_num)

                    Part_Q_start = time.time()
                    makespan_Q_Part = QLearning(timespace_G, timespace_s, timespace_goal, nodenum_onelayer, 100, 0.9, 0.1, temp_path_dict)
                    Part_Q_time_list.append(time.time() - Part_Q_start)
                    if(makespan_Q_Part <1000):
                        Part_Q_res.extend([makespan_Q_Part,sum(Part_Q_time_list)])
                        Part_Q_suc = True
                        print('Part Q suc')
                        break
        except:
            print('PartQ_Fail')

        if(Part_Q_suc == True and Q_suc ==True):
            break


    return([Q_res,Part_Q_res])

if __name__ == '__main__':
    #-----------------------test Sample--------------------------
    # Grid_size = 2
    # Obs_ratio = 0
    # Robot_num = 2
    #
    # Arr = GenerateMap(Grid_size,Obs_ratio,Robot_num)
    # D1_map = Arr.reshape(-1)
    # nodenum_onelayer = len(D1_map)
    #
    # # ILP
    # try:
    #     makespan_ILP, time_ILP = Main_ILP(Grid_size,Obs_ratio,Robot_num,Arr)
    #     print(makespan_ILP,time_ILP)
    # except:
    #     print('cannot find')
    #
    #
    # #Q_learning
    # try:
    #     Q_res,Part_Q_res = Main_Q(Grid_size,Obs_ratio,Robot_num,Arr)
    #     makespan_Q = Q_res[0]
    #     time_Q = Q_res[1]
    #     Part_makespan_Q = Part_Q_res[0]
    #     Part_time_Q = Part_Q_res[1]
    #     print('Q:',makespan_Q,time_Q)
    #     print('PartQ:',Part_makespan_Q,Part_time_Q)
    # except:
    #     print('Q_learningFalse')

    #-------------------Experiment----------------------------
    Experiment_start = time.time()
    makespan_ILP_sum = 0
    makespan_partQ_sum = 0
    makespan_Q_sum = 0

    time_ILP_sum = 0
    time_partQ_sum = 0
    time_Q_sum = 0

    ILP_succ = 0
    partQ_succ = 0
    Q_succ = 0

    ILP_exceed_count = 0
    partQ_exceed_count = 0
    Q_exceed_count = 0

    Grid_size = 5
    Obs_ratio = 0.2
    Robot_num = 5
    time_exceed_Threshold = 30

    for i in range(0,100):
        Arr = GenerateMap(Grid_size, Obs_ratio, Robot_num)
        D1_map = Arr.reshape(-1)
        nodenum_onelayer = len(D1_map)
        print(Arr)
        print('round:',i)
        #ILP
        try:
            makespan_ILP, time_ILP = Main_ILP(Grid_size,Obs_ratio,Robot_num,Arr)
            #print(makespan_ILP, time_ILP)
            if(time_ILP<= time_exceed_Threshold):
                ILP_succ = ILP_succ + 1
                makespan_ILP_sum = makespan_ILP_sum + makespan_ILP
                time_ILP_sum = time_ILP_sum + time_ILP
            else:
                ILP_exceed_count = ILP_exceed_count+1
        except:
            print('cannot find ILP')

        #Q-Learning

        Q_res,Part_Q_res = Main_Q(Grid_size,Obs_ratio,Robot_num,Arr)
        #Q
        try:
            makespan_Q = Q_res[0]
            time_Q = Q_res[1]
            #print('Q:', makespan_Q, time_Q)
            if(time_Q <= time_exceed_Threshold):
                Q_succ = Q_succ + 1
                makespan_Q_sum = makespan_Q_sum + makespan_Q
                time_Q_sum = time_Q_sum + time_Q
            else:
                Q_succ = Q_succ + 1
                time_Q_sum = time_Q_sum + time_Q
                makespan_Q_sum = makespan_Q_sum + makespan_Q
                Q_exceed_count = Q_exceed_count + 1
        except:
            print('Final Q Fasle')

        #Partial Q-Learning
        try:
            Part_makespan_Q = Part_Q_res[0]
            Part_time_Q = Part_Q_res[1]
            #print('PartQ:', Part_makespan_Q, Part_time_Q)
            if(Part_time_Q <= time_exceed_Threshold):
                partQ_succ = partQ_succ + 1
                makespan_partQ_sum = makespan_partQ_sum + Part_makespan_Q
                time_partQ_sum = time_partQ_sum + Part_time_Q
            else:
                partQ_succ = partQ_succ + 1
                makespan_partQ_sum = makespan_partQ_sum + Part_makespan_Q
                time_partQ_sum = time_partQ_sum + Part_time_Q
                partQ_exceed_count = partQ_exceed_count + 1
        except:
            print('Final Part Q Fasle')


    Experiment_time = time.time() - Experiment_start
    mail(str(Experiment_time))

    filename = "5_29G" + str(Grid_size) + "O" + str(Obs_ratio) + "R" + str(Robot_num)
    with open("Experiment_Data/new/" + filename + ".txt", "w") as f:
        f.write('ILP\n')
        f.write('ILP Makespan:' + str(makespan_ILP_sum / ILP_succ) + '\n')
        f.write('ILP time:' + str(time_ILP_sum / ILP_succ) + '\n')
        f.write('ILP success:' + str(ILP_succ) + '\n')
        f.write('ILP timeout:' + str(ILP_exceed_count) + '\n')

        if(partQ_succ == 0):
            f.write('Partial Q-Learning_partQ=0\n')
            f.write('PartQ makespan:' + str(makespan_partQ_sum) + '\n')
            f.write('PartQ time:' + str(time_partQ_sum) + '\n')
            f.write('PartQ success:' + str(partQ_succ) + '\n')
            f.write('PartQ timeout:' + str(partQ_exceed_count) + '\n')
        else:
            f.write('Partial Q-Learning\n')
            f.write('PartQ makespan:' + str(makespan_partQ_sum / partQ_succ) + '\n')
            f.write('PartQ time:' + str(time_partQ_sum / partQ_succ) + '\n')
            f.write('PartQ success:' + str(partQ_succ) + '\n')
            f.write('PartQ timeout:' + str(partQ_exceed_count) + '\n')
        if(Q_succ==0):
            f.write('Q-Learning=0\n')
            f.write('Q makespan:' + str(makespan_Q_sum) + '\n')
            f.write('Q time:' + str(time_Q_sum) + '\n')
            f.write('Q success:' + str(Q_succ) + '\n')
            f.write('Q_timeout:' + str(Q_exceed_count) + '\n')
        else:
            f.write('Q-Learning\n')
            f.write('Q makespan:' + str(makespan_Q_sum / Q_succ) + '\n')
            f.write('Q time:' + str(time_Q_sum / Q_succ) + '\n')
            f.write('Q success:' + str(Q_succ) + '\n')
            f.write('Q_timeout:' + str(Q_exceed_count) + '\n')
        f.write('\n')
        f.write('Experiment_time:'+str(Experiment_time))

    print(Experiment_time)
    # mail(str(Experiment_time))
