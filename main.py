# coding=UTF-8
import networkx as nx
import io
import yaml
import copy
import matplotlib.pyplot as plt # 导入 Matplotlib 工具包
from itertools import combinations, product
from heapq import heappush, heappop
from itertools import count

# https://blog.csdn.net/u010330109/article/details/89525729
# 它这个思路是求所有不连通的子图，那么可以吧\Sigma_o当成断开的进行求解即可，求完加上去

max_edge_number = 3

event_uo = ['a',  'b',  'uc']
event_o  = ['o1', 'o2', 'o3']
event_c  = ['a',  'b',  'o3']
event_uc = ['o1', 'o2', 'uc']

def dfs_edges(G, event_list, source=None, depth_limit=None):
    if source is None:
        # edges for all components
        nodes = G
    else:
        # edges for components with source
        nodes = [source]
    visited = set()
    if depth_limit is None:
        depth_limit = len(G)
    for start in nodes:
        if start in visited:
            continue
        visited.add(start)
        stack = [(start, depth_limit, iter(G[start]))]
        while stack:
            parent, depth_now, children = stack[-1]
            try:
                child = next(children)

                if G.edges[parent, child, 0]['event'] not in event_list:  # added
                    continue

                if str([parent, child]) not in visited:     # 因为list本身不可哈希，所以用str(list())来代替list
                    yield parent, child                     # yield parent, child 这个版本的python没法调试yield  https://zhuanlan.zhihu.com/p/268605982
                    visited.add(str([parent, child]))       # visited.add(child)
                    if depth_now > 1:
                        stack.append((child, depth_now - 1, iter(G[child])))

            except StopIteration:
                stack.pop()

def dfs_events(iwa, event_list, source):
    edges = list(dfs_edges(iwa, event_list, source))

    G0 = nx.MultiDiGraph()

    for edge_t in edges:
        start = list(edge_t)[0]
        end   = list(edge_t)[1]
        try:
            event = iwa.edges[start, end, 0]['event']
            if event in event_list:                                 # 计算路径长的时候不能过可观事件
                t_min =  iwa.edges[start, end, 0]['t_min']
                t_max = -iwa.edges[start, end, 0]['t_max']          # 用负值，得到的最短距离就是最长距离
                G0.add_edge(start, end, event=event, t_min=t_min, t_max=t_max)
        except:
            pass

    # 这里用了个笨办法
    # 因为真正得到的dfs_tree的t_min和t_max要是累计值，而且累计值必须从取出的edge里面出
    # 所以这里先用取出的edge建了个图，然后在这个图里面做最短/最长路径
    dfs_tree = nx.MultiDiGraph()
    for edge_t in edges:
        start = list(edge_t)[0]
        end   = list(edge_t)[1]
        try:
            event = iwa.edges[start, end, 0]['event']

            if event not in event_list:                                                                                                      # 这个是之前的版本遗留下来的，那个时候希望dfs_tree到一个可观事件然后停止，所以需要这么计算
                G0.add_edge(start, end, event=event, t_min=iwa.edges[start, end, 0]['t_min'], t_max=-iwa.edges[start, end, 0]['t_max'])      # 这个操作很暴力，因为如果要算过可观事件的min_max距离，那只有这条可观边是可走的，那么就在计算的时候加进来，算完就删掉
                #t_min =  nx.shortest_path_length(G0, source, end, weight='t_min')
                #t_max = -nx.shortest_path_length(G0, source, end, weight='t_max')
                t_min  = nx.shortest_path_length(G0, source, start, weight='t_min') + iwa.edges[start, end, 0]['t_min']       # 避免计算结果为0，比如从6→6
                t_max = -nx.shortest_path_length(G0, source, start, weight='t_max') + iwa.edges[start, end, 0]['t_max']       # 其实start→end是必经边，这个信息其实很关键
                G0.remove_edge(start, end)
            else:
                #t_min =  nx.shortest_path_length(G0, source, end, weight='t_min')
                #t_max = -nx.shortest_path_length(G0, source, end, weight='t_max')
                t_min  = nx.shortest_path_length(G0, source, start, weight='t_min') + iwa.edges[start, end, 0]['t_min']
                t_max = -nx.shortest_path_length(G0, source, start, weight='t_max') + iwa.edges[start, end, 0]['t_max']

            dfs_tree.add_edge(start, end, event=event, t_min=t_min, t_max=t_max)
        except:
            pass

    # 到这里计算到的都是通过uo到达的最短路径
    # 那些可经由可达时间到达的点还没有做出来
    for edge_t in edges:
        start = list(edge_t)[0]
        end   = list(edge_t)[1]

    return dfs_tree

def timeslice(dfs_tree):
    t_interval = []
    for edge_t in  dfs_tree.edges:
        t_min = dfs_tree.edges[list(edge_t)[0], list(edge_t)[1], list(edge_t)[2]]['t_min']
        t_max = dfs_tree.edges[list(edge_t)[0], list(edge_t)[1], list(edge_t)[2]]['t_max']
        t_interval.append(t_min)
        t_interval.append(t_max)

    t_interval = list(set(t_interval))      # 排序，去除多余元素
    return t_interval

def get_event(iwa, start, end):
    return iwa.edges[start, end, 0]['event']

def get_t_min(iwa, start, end):
    return iwa.edges[start, end, 0]['t_min']

def get_t_max(iwa, start, end):
    return iwa.edges[start, end, 0]['t_max']

def dfs_ur(dfs_tree, sc, source=None, depth_limit=None):
    if source is None:
        # edges for all components
        nodes = dfs_tree
    else:
        # edges for components with source
        nodes = [source]
    visited = set()
    if depth_limit is None:
        depth_limit = len(dfs_tree)
    for start in nodes:
        if start in visited:
            continue
        visited.add(start)
        stack = [(start, depth_limit, iter(dfs_tree[start]))]
        while stack:
            parent, depth_now, children = stack[-1]
            try:
                child = next(children)

                is_edge_reachable = False         # added
                for sc_t in sc:
                    event_t = list(sc_t)[0]
                    t = list(sc_t)[1]
                    if event_t == dfs_tree.edges[parent, child, 0]['event'] and dfs_tree.edges[parent, child, 0]['t_min'] <= t:
                        is_edge_reachable = True
                        break
                if not is_edge_reachable:
                    continue

                if str([parent, child]) not in visited:     # 因为list本身不可哈希，所以用str(list())来代替list
                    yield parent, child                     # yield parent, child 这个版本的python没法调试yield  https://zhuanlan.zhihu.com/p/268605982
                    visited.add(str([parent, child]))       # visited.add(child)
                    if depth_now > 1:
                        stack.append((child, depth_now - 1, iter(dfs_tree[child])))

            except StopIteration:
                stack.pop()

def state_type(state):
    if type(state[0]) == tuple and list(state).__len__() == 2:      ## 先用2，后面要记得改成3，判断方式也要细改一下
        return 'Z_state'
    else:
        return 'Y_state'

def get_policy_event(state):
    policy_events = []
    for policy_t in state[1]:       ## 后面要改成state[2]
        policy_events.append(policy_t[0])
    return policy_events

def get_policy_t_min(state):
    policy = {}
    for policy_t in state[1]:       ## 后面要改成state[2]
        policy.update({policy_t[0] : policy_t[1]})
    return policy
def get_policy_duration(state):
    policy = {}
    for policy_t in state[1]:       ## 后面要改成state[2]
        policy.update({policy_t[0] : (policy_t[1], policy_t[2])})
    return policy

def get_min_max_time_from_y(iwa, y_state, current_node):
    t_min = 1e6  # 这也是个min-max结构，对每一个y状态求解y->z->y的最短路径
    t_max = -1
    G0 = nx.MultiDiGraph()
    for edge_g in iwa.edges():
        start = list(edge_g)[0]
        end = list(edge_g)[1]
        try:
            event = iwa.edges[start, end, 0]['event']
            t_min = iwa.edges[start, end, 0]['t_min']
            t_max = -iwa.edges[start, end, 0]['t_max']  # 用负值，得到的最短距离就是最长距离
            G0.add_edge(start, end, event=event, t_min=t_min, t_max=t_max)
        except:
            pass

    for node_s in y_state:
        try:
            t_min_t = nx.shortest_path_length(G0, node_s, current_node, weight='t_min', method='bellman-ford')
            if t_min_t < t_min:
                t_min = t_min_t
            t_max_t = -nx.shortest_path_length(G0, node_s, current_node, weight='t_max', method='bellman-ford')     ## 现在是这个搞不定
            if t_max_t > t_max:
                t_max = t_max_t
        except:
            pass

    return [t_min, t_max]

def t_aic(iwa, source, event_uo, event_o, event_c, event_uc):
    # 把初始点设置为Y state
    bts = nx.MultiDiGraph()
    bts.add_node(tuple(source))
    bts_start = tuple(source)

    event_uo_c  = set(event_uo) & set(event_c)          # & 求交集
    event_uo_uc = set(event_uo) & set(event_uc)

    # 先求可控事件的全子集
    events_2c = []
    for index in range(list(event_c).__len__()):
        for subset_t in combinations(list(event_c), index + 1):
            events_2c.append(list(subset_t))
    events_2uo_uc = []
    for index in range(list(event_uo_uc).__len__()):
        for subset_t in combinations(list(event_uo_uc), index + 1):
            events_2uo_uc.append(list(subset_t))

    # 因为不可观不可控事件可以认为在supervisior中强制打开的，所以直接加入就好了，这里用笛卡尔积的形式直接算到了所有事件的组合形式
    supervisior_ut = []
    for _iter in product(list(events_2c), list(events_2uo_uc)):
        _iter = list(_iter)[0] + list(_iter)[1]
        supervisior_ut.append(_iter)


    ###################################################################################################################
    #
    # 这一部分应该才开始迭代

    # 核心思路：Z->Z的状态，可以通过dfs_tree全部求出
    #         所以我们只要求出下一时刻的Y状态
    #         同时，应注意，到达Y状态，supervisor清零，所以其实不用特别在意时间，只是求NX(·)的时候注意时间就好了
    # 思路：
    # 1 对Y state的每一个点求dfs_tree
    # 2
    # 结束条件：没有新的y_state可生成
    y_stack = []
    visited = []

    y_stack.append(tuple(bts_start))
    visited.append(tuple(bts_start))
    while y_stack:
        # 编程的时候你就想着，如果这个自动机有两个入口你怎么办
        current_state = y_stack.pop()

        for sc in supervisior_ut:
            t_interval = []

            max_time_uo = {}
            min_time_uo = {}
            sc_event_tuple = []
            for event_t in list(sc):
                max_time_uo.update({event_t: -1})
                min_time_uo.update({event_t: 1e6})
            for current_node in current_state:
                dfs_tree = dfs_events(iwa, sc, current_node)                    ## 待增加对环状结构的适应性
                t_interval = list(set(t_interval) | set(timeslice(dfs_tree)))

                # 求出不同事件对应的时间的最大值
                for u, v, data in dfs_tree.out_edges(data=True):                 # u, v, data in dfs_tree.out_edges(curr_node, data=True)
                    if data['event'] in sc and data['t_max'] > max_time_uo[event_t]:
                        max_time_uo[data['event']] = data['t_max']
                    if data['event'] in sc and data['t_min'] < min_time_uo[event_t]:
                        min_time_uo[data['event']] = data['t_min']              # 用了个笨办法来处理可观可控事件，因为它不能往下走，所以对他的使能时间有个限制

            # 从timeslice内选取不超过时间最大值的时间，给对应事件
            for _iter in max_time_uo.keys():
                sc_timed_t = []
                for t in t_interval:
                    if _iter not in event_uo and t >= min_time_uo[_iter] and t <= max_time_uo[_iter]:
                        sc_timed_t.append((_iter, t))
                    elif _iter in event_uo and t <= max_time_uo[_iter]:         # 如果不加 _iter in event_uo 会出错
                        sc_timed_t.append((_iter, t))
                if sc_timed_t.__len__() != 0:
                    sc_event_tuple.append(sc_timed_t)       # 加入非空元素

            sc_set = list(product(*sc_event_tuple))                 # https://blog.csdn.net/weixin_39652760/article/details/110774080
                                                                    # https://blog.csdn.net/liuqiang3/article/details/99707294
            sc_set = [list(_iter) for _iter in sc_set]              # tuple -> list
                                                                    # 而且最后拿到的数据是排序排好的，这里就不用考虑连接问题
            sc_set.sort()                                           # Critical

            #last_state = current_state
            for supervisior_curr in sc_set:
                ur = []
                ur_new = []

                #if ('b', 9) in supervisior_curr:    # current_state == ('6',)
                #    print(233)

                for current_node in current_state:
                    try:
                        sc = [sc_ut[0] for sc_ut in supervisior_curr]
                        dfs_tree = dfs_events(iwa, sc, current_node)
                        reachable_edge = list(dfs_ur(dfs_tree, supervisior_curr, source=current_node))

                        # edge -> 可达点
                        ur.append(current_node)
                        for edge_t in reachable_edge:
                            if dfs_tree.edges[edge_t[0], edge_t[1], 0]['event'] in event_uo:            # 只有不可观边能到达的才是ur,至于为什么不在dfs里处理，那是因为这么做会影响得到的决策数据
                                ur.append(list(edge_t)[1])                                              # 可以发生transition的终点都是可达的点
                        ur = list(set(ur))
                        ur.sort()
                    except KeyError:
                        #ur.append(current_node)                                                         ## 5(Y)->5(Z)->5(Y)
                        pass

                if ur.__len__() == 0:
                    z_state = (tuple(current_state), tuple(supervisior_curr))
                else:
                    z_state = (tuple(ur), tuple(supervisior_curr))
                if z_state == ((), ()):
                    continue

                is_state_listed = False
                is_state_listed_in_this_y = False
                for state_t in bts.nodes():
                    if state_type(z_state) == state_type(state_t) and \
                            state_t[0] == z_state[0] and \
                            set(get_policy_event(state_t)) == set(get_policy_event(z_state)):
                        is_state_listed = True
                        try:
                            if nx.dijkstra_path_length(bts, current_state, state_t) >= 0:
                                is_state_listed_in_this_y = True
                                break
                        except:
                            pass

                if is_state_listed and is_state_listed_in_this_y:           # 如果这个点已经有过
                    for edge_t in bts.in_edges(state_t, data=True):
                        # 更新点
                        # bts.remove_node(edge_t[0])
                        # bts.add_node(z_state)

                        # 更新边
                        # bts.remove_node(edge_t[0], edge_t[1])
                        sc_last = bts.edges[edge_t[0], edge_t[1], 0]['control']

                        event_last_list = []
                        for sc_timed_last in sc_last:
                            event_last_list.append(list(sc_timed_last)[0])

                        for sc_timed_t in supervisior_curr:
                            event_t = list(sc_timed_t)[0]
                            for sc_timed_last in sc_last:
                                event_last = list(sc_timed_last)[0]

                                # 如果这一个控制是存在的，那么则更新，扩大其时间区间
                                if event_t in event_last_list and event_t != event_last:
                                    continue
                                elif event_t in event_last_list and event_t == event_last:
                                    index = event_last_list.index(event_last)

                                    start_t = bts.edges[edge_t[0], edge_t[1], 0]['control'][index][1]
                                    if t_interval.index(list(sc_timed_t)[1]) == t_interval.__len__() - 1:
                                        end_t = float("inf")
                                    else:
                                        end_t = t_interval[t_interval.index(list(sc_timed_t)[1]) + 1]
                                    if end_t > bts.edges[edge_t[0], edge_t[1], 0]['control'][index][2]:
                                        bts.edges[edge_t[0], edge_t[1], 0]['control'][index] = (event_t, start_t, end_t)
                                    # 否则加入该控制
                                # 这里不能允许新加入，新加入的就不是一个控制了
                                '''
                                else:
                                    event_t = list(sc_timed_t)[0]
                                    start_t = list(sc_timed_t)[1]
                                    if t_interval.index(list(sc_timed_t)[1]) == t_interval.__len__() - 1:
                                        end_t = float("inf")
                                    else:
                                        end_t = t_interval[t_interval.index(list(sc_timed_t)[1]) + 1]
                                    bts.edges[edge_t[0], edge_t[1], 0]['control'].append((event_t, start_t, end_t))
                                    event_last_list.append(event_t)
                                    #print(466)
                                '''
                #elif is_state_listed and not is_state_listed_in_this_y:     # 如果这个点对于当前y来说是全新的
                #    print(4666)
                #    pass
                else:
                    # 这一段都是为了再算边的信息, add_edge这里才决定了根节点
                    sc_duration = []
                    for sc_timed_t in supervisior_curr:
                        event_t = list(sc_timed_t)[0]
                        start_t = list(sc_timed_t)[1]
                        if t_interval.index(list(sc_timed_t)[1]) == t_interval.__len__() - 1:
                            end_t = float("inf")
                        else:
                            end_t = t_interval[t_interval.index(list(sc_timed_t)[1]) + 1]
                        sc_duration.append((event_t, start_t, end_t))

                    root_state = current_state                  # 如果点找不到，就是一定连接当前y_state的
                    event_tz = get_policy_t_min(z_state)
                    t_max_t = copy.deepcopy(event_tz)
                    for _iter in t_max_t.keys():
                        t_max_t[_iter] = -1                     # 初始化，求最大时间

                    for state_t in bts.nodes():                 # 找到 decision中，满足如下条件的点：1 事件完全相符 2 事件对应使能时间都比z_state小 3 满足1、2中，每箱使能时间对应最长
                        if not state_type(state_t) == 'Z_state':
                            continue
                        event_t  = get_policy_t_min(state_t)
                        if event_t.keys() == event_tz.keys():           # 1 事件完全相符
                            iter_num = 0
                            for _iter in event_t.keys():
                                if event_t[_iter] <= event_tz[_iter]:   # 2 所有事件对应使能时间都比z_state小
                                    iter_num += 1
                            if iter_num == event_t.__len__():
                                try:
                                    #print(event_t, '\t', event_tz)
                                    if nx.dijkstra_path_length(bts, current_state, state_t) >= 0:   # 4 要求：找到的点和当前要增加的点都必须是在当前同一y状态之下
                                        root_state = state_t
                                        for _iter in event_t.keys():
                                            t_max_t[_iter] = event_t[_iter]     # 3 满足1、2中，每箱使能时间对应最长
                                except:
                                    pass

                    # 增加点
                    if not is_state_listed:
                        z_state = (z_state[0], tuple(sc_duration))
                        bts.add_node(z_state)
                    # 增加边
                    bts.add_edge(root_state, z_state, control=sc_duration)
                    # 这里是将边，当前点，与根节点相连
                    #bts.add_edge(last_state, z_state, control=sc_duration)

                    # ITERATION
                    #last_state = z_state  # 迭代更新，因为前面edge都是排好的，所以这里直接加进来


        # 求NX
        # 现有思路：  针对每一个Z状态，求下一状态
        #           发现一个接一个，直接接在当前遍历的Z状态上
        #           问题就出在这边 发现一个接一个 ，这样得到的状态必然是分散的
        # 解决思路：
        state_to_add = []
        edge_to_add  = []
        for state_t in bts.nodes():
            # 对所有z_states 求NX
            if state_t not in visited and state_type(state_t) == 'Z_state':
                y_state_w_observations = []     # 同时存放事件和

                # 这边我是希望得到所有事件相同的点，最后再一起处理
                for node_t in state_t[0]:
                    for edge_t in iwa.out_edges(node_t, data=True):
                        if edge_t[2]['event'] in event_o:     # 如果存在可到达的事件

                            for current_node in current_state:
                                dfs_tree = dfs_events(iwa, get_policy_event(state_t), current_node)
                                if not (node_t in dfs_tree.nodes() or current_node in dfs_tree.nodes()):
                                    continue

                                feasible_path = list(nx.all_simple_paths(dfs_tree, current_node, node_t))
                                if feasible_path.__len__() == 0:                                            # 如果没有可达路径，则重新查找，怎么可能没有可达路径
                                    continue

                                t_min = 1e6
                                t_max = -1
                                for path_t in feasible_path:
                                    last_node = path_t[path_t.__len__() - 2]
                                    t_min_t = dfs_tree.edges[last_node, node_t, 0]['t_min']                 # 因为dfs_tree里面会算好累加长度，所以这里只需调用
                                    t_max_t = dfs_tree.edges[last_node, node_t, 0]['t_max']
                                    if t_min_t < t_min:
                                        t_min = t_min_t
                                    if t_max_t > t_max:
                                        t_max = t_max_t

                                if edge_t[2]['event'] not in event_c or \
                                   (edge_t[2]['event'] in event_c and edge_t[2]['event'] in get_policy_event(state_t) and \
                                        get_policy_duration(state_t)[edge_t[2]['event']][0] >= t_min + edge_t[2]['t_min']):     # 可观不可控，或：可观可控且被使能，且使能时间大于当前结点的最小时间
                                                                                                                                # 对可观可控事件，要求当前事件的使能最小时间要大于当前的最小观测时间
                                    # TODO：求解此处min-max时间
                                    t_min += edge_t[2]['t_min']
                                    t_max += edge_t[2]['t_max']

                                    y_state_w_observations.append((state_t, edge_t[0], edge_t[1], (edge_t[2]['event'], t_min, t_max)))      # 当前状态，当前状态中出发点，当前状态可达点，事件以及花的时间

                if y_state_w_observations.__len__():

                    # 整理每一步得到的y_state
                    # Step 1: 取出所有事件和对应时间，做timeslice
                    nx_timeslice = {}
                    for y_t in y_state_w_observations:
                        if y_t[3][0] not in nx_timeslice.keys():            # y_t[3][0] 当前可观事件的事件名，如果当前可观事件未被记录
                            nx_timeslice.update({y_t[3][0]: [y_t[3][1], y_t[3][2]]})
                        else:
                            nx_timeslice[y_t[3][0]].append(y_t[3][1])
                            nx_timeslice[y_t[3][0]].append(y_t[3][2])

                        nx_timeslice.update({y_t[3][0]: list(set(nx_timeslice[y_t[3][0]]))})        # 去除相同项
                        nx_timeslice[y_t[3][0]].sort()                                              # 排序

                    # Step 2: 对每一时间段，求解当前的可达点
                    for event_t in nx_timeslice:
                        for i in range(0, nx_timeslice[event_t].__len__() - 1):
                            t1 = nx_timeslice[event_t][i]                                                   # 取出相邻的时间
                            t2 = nx_timeslice[event_t][i + 1]

                            state_to_add_t = []
                            for y_t in y_state_w_observations:
                                if y_t[3][0] == event_t and (y_t[3][1] <= t1 and y_t[3][2] >= t2):          # 当前遍历到的observation事件相等
                                    if y_t[2] not in state_to_add_t:
                                        state_to_add_t.append(y_t[2])                                       # 把可达点加入

                            state_to_add_t.sort()                                                           # 排序
                            if tuple(state_to_add_t) not in state_to_add:                                   # 对于当前事件event_t和t1, t2，所有可达点组成新的y状态
                                state_to_add.append(tuple(state_to_add_t))                                  # 如果该状态之前没被考虑过，加入

                                visited.append(tuple(state_to_add_t))
                                y_stack.append(tuple(state_to_add_t))

                            if t1 == 13 and t2 == 16:
                                print(233)

                            is_edge_added = False
                            index_ea = None
                            for edge_to_add_t in edge_to_add:
                                if edge_to_add_t[0] == y_t[0] and edge_to_add_t[1] == tuple(state_to_add_t) and \
                                   event_t == edge_to_add_t[2][0] and \
                                   (t1 == edge_to_add_t[2][2] or t2 == edge_to_add_t[2][1]):                # 关键，时间一定要相接
                                    is_edge_added = True
                                    index_ea = edge_to_add.index(edge_to_add_t)
                            if not is_edge_added:
                                edge_to_add.append((y_t[0], tuple(state_to_add_t), (event_t, t1, t2)))      # 如果当前边没有被考虑过，则添加
                                visited.append(y_t[0])
                            else:                                                                           # 如果有起点相同，终点相同，且时间相接的边，则合并，例子见下
                                t1 = min(t1, edge_to_add[index_ea][2][1])                                   # ('o1', 7, 13) ('o1', 13, 16))
                                t2 = max(t2, edge_to_add[index_ea][2][2])
                                edge_to_add[index_ea] = (y_t[0], tuple(state_to_add_t), (event_t, t1, t2))

        for index in range(0, state_to_add.__len__()):      # dictionary changed size during iteration
            try:
                bts.add_node(state_to_add[index])
            except:
                pass
        for index in range(0, edge_to_add.__len__()):      # dictionary changed size during iteration
            bts.add_edge(edge_to_add[index][0], edge_to_add[index][1], observtion=edge_to_add[index][2])


    return bts

def assign_node_colors(bts):
    values = []
    for node_t in bts.nodes():
        if node_t == ('init', ):
            values.append('#DC5712')                # https://www.sioe.cn/yingyong/yanse-rgb-16/
        elif state_type(node_t) == 'Z_state':
            values.append('#FE4365')                # https://www.icoa.cn/a/512.html
        else:
            values.append('#83AF9B')
    return values

def main():
    fin = open('./IWA_1.yaml', 'r', encoding='utf-8')
    data = yaml.load(fin, Loader=yaml.FullLoader)

    iwa = nx.MultiDiGraph()       # Graph MultiGraph
    for node_t in data['graph']['nodes']:
        iwa.add_node(node_t)
    for edge_t in data['graph']['edges']:
        event = edge_t[2]['event']
        t_min = edge_t[2]['t_min']
        t_max = edge_t[2]['t_max']
        iwa.add_edge(edge_t[0], edge_t[1], event=event, t_min=t_min, t_max=t_max)

    # 建立一个不可观事件的可达树
    dfs_tree = dfs_events(iwa, ['b'], source='0')

    # 求出dfs_tree对应的所有时间点
    #t_interval = timeslice(dfs_tree)

    init_state = ['0']                                                  # ['0', '6'], ['6']
    bts = t_aic(iwa, init_state, event_uo, event_o, event_c, event_uc)  # iwa, ['0', '6'], event_uo, event_o, event_c, event_uc

    '''
        Plotting
    '''
    '''
    # https://www.yht7.com/
    # https://www.yht7.com/news/144953#:~:text=%E5%AF%B9%E4%BA%8E%E6%97%A0%E9%99%90%E5%88%B6%E6%9D%A1%E4%BB%B6%E7%9A%84%E6%9C%80%E7%9F%AD%E8%B7%AF%E5%BE%84%E9%97%AE%E9%A2%98%EF%BC%8CNetworkX%20%E6%8F%90%E4%BE%9B%E4%BA%86%20Dijkstra%20%E7%AE%97%E6%B3%95%E3%80%81Bellman-Ford%20%E7%AE%97%E6%B3%95%E3%80%81Floyd%20%E7%AE%97%E6%B3%95%E5%92%8C%E5%90%AF%E5%8F%91%E5%BC%8F%E7%AE%97%E6%B3%95%20A%2A%20%E7%9A%84%E5%87%BD%E6%95%B0%E3%80%82,nx.dijkstra_path%20%28%29%20%E5%92%8C%20nx.dijkstra_path_length%20%28%29%20%E8%B0%83%E7%94%A8%20Dijkstra%20%E7%AE%97%E6%B3%95%E6%B1%82%E4%B8%A4%E4%B8%AA%E6%8C%87%E5%AE%9A%E9%A1%B6%E7%82%B9%E4%B9%8B%E9%97%B4%E7%9A%84%E6%9C%80%E7%9F%AD%E5%8A%A0%E6%9D%83%E8%B7%AF%E5%BE%84%E5%92%8C%E6%9C%80%E7%9F%AD%E5%8A%A0%E6%9D%83%E8%B7%AF%E5%BE%84%E9%95%BF%E5%BA%A6%E3%80%82
    pos = { '0' : (0.5, 1),
            '1' : (0.2, 0.8),
            '2' : (0.2, 0.6),
            '3' : (0.2, 0.4),
            '4' : (0.5, 0.8),
            '5' : (0.5, 0.6),
            '6' : (0.5, 0.4),
            '7' : (0.8, 0.8),
            '8' : (0.8, 0.6),
            '9' : (0.8, 0.4)}

    nx.draw(dfs_tree, pos, with_labels=True, alpha=0.8)

    labels = {}
    for edge_t in dfs_tree.edges():
        #print(edge_t)
        min_t = dfs_tree.edges[list(edge_t)[0], list(edge_t)[1], 0]['t_min']
        max_t = dfs_tree.edges[list(edge_t)[0], list(edge_t)[1], 0]['t_max']
        labels.update({edge_t : str('[' + str(min_t) + ', ' + str(max_t) + ']')})
    nx.draw_networkx_edge_labels(dfs_tree, pos, edge_labels=labels, font_color="c")  # 显示权值
    '''
    bts.add_edge(('init', ), tuple(init_state))
    node_color = assign_node_colors(bts)

    pos = nx.shell_layout(bts)   # nx.spectral_layout(bts)
    nx.draw(bts, pos=pos, with_labels=True, node_color=node_color, font_size=8.5)                        # https://www.jianshu.com/p/e254cd6acfdc/
                                                                                                         # https://blog.csdn.net/HsinglukLiu/article/details/107821649
                                                                                                         # https://www.cnpython.com/qa/39393

    nx.draw_networkx_edge_labels(bts, pos, font_size=4.5)                                                # https://blog.csdn.net/u013576018/article/details/60871485

    plt.show()

if __name__ == '__main__':
    main()
    print('Finished')
