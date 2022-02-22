# coding=UTF-8
import networkx as nx
import io
import yaml
import copy
import matplotlib.pyplot as plt # 导入 Matplotlib 工具包
from itertools import combinations, product

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

def get_policy_dict(state):
    policy = {}
    for policy_t in state[1]:       ## 后面要改成state[2]
        policy.update({policy_t[0] : policy_t[1]})
    return policy


def t_aic_onestep(iwa, source, event_uo, event_o, event_c, event_uc):
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

            #last_state = current_state
            for supervisior_curr in sc_set:
                ur = []
                ur_new = []

                try:
                    for current_node in current_state:
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

                    z_state = (tuple(ur), tuple(supervisior_curr))

                    is_state_listed = False
                    for state_t in bts.nodes():
                        if state_type(z_state) == state_type(state_t) and \
                                state_t[0] == z_state[0] and \
                                set(get_policy_event(state_t)) == set(get_policy_event(z_state)):
                            is_state_listed = True
                            break

                    if is_state_listed:
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
                                        # print(233)
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
                        event_tz = get_policy_dict(z_state)
                        t_max_t = copy.deepcopy(event_tz)
                        for _iter in t_max_t.keys():
                            t_max_t[_iter] = -1                     # 初始化，求最大时间

                        for state_t in bts.nodes():                 # 找到 decision中，满足如下条件的点：1 事件完全相符 2 事件对应使能时间都比z_state小 3 满足1、2中，每箱使能时间对应最长
                            if not state_type(state_t) == 'Z_state':
                                continue
                            event_t  = get_policy_dict(state_t)
                            if event_t.keys() == event_tz.keys():           # 1 事件完全相符
                                iter_num = 0
                                for _iter in event_t.keys():
                                    if event_t[_iter] <= event_tz[_iter]:   # 2 所有事件对应使能时间都比z_state小
                                        iter_num += 1
                                if iter_num == event_t.__len__():
                                    #print(event_t, '\t', event_tz)
                                    root_state = state_t
                                    for _iter in event_t.keys():
                                        t_max_t[_iter] = event_t[_iter]     # 3 满足1、2中，每箱使能时间对应最长

                        # 增加点
                        bts.add_node(z_state)
                        # 增加边
                        bts.add_edge(root_state, z_state, control=sc_duration)
                        # 这里是将边，当前点，与根节点相连
                        #bts.add_edge(last_state, z_state, control=sc_duration)

                        # ITERATION
                        #last_state = z_state  # 迭代更新，因为前面edge都是排好的，所以这里直接加进来

                except KeyError:
                    pass

            '''
            for current_node in current_state:
                print(233)
            '''

        # 求NX
        state_to_add = []
        edge_to_add  = []
        for state_t in bts.nodes():
            # 对所有z_states 求NX
            if state_t not in visited and state_type(state_t) == 'Z_state':
                y_state_t = []
                for node_t in state_t[0]:
                    for edge_t in iwa.out_edges(node_t, data=True):
                        if edge_t[2]['event'] in event_o and edge_t[2]['event'] not in event_c:
                            y_state_t.append(edge_t[1])                                             ## 时间要算过

                        elif edge_t[2]['event'] in event_o and edge_t[2]['event'] in event_c:
                            pass

                if y_state_t.__len__() > 0 and tuple(y_state_t) not in y_stack:
                    visited.append(y_state_t)
                    visited.append(state_t)

                    y_stack.append(tuple(y_state_t))
                    state_to_add.append(tuple(y_state_t))
                    edge_to_add.append([state_t, tuple(y_state_t)])                                      ## 这里edge的数据不知道

        print(y_stack, len(bts.nodes()))

        for index in range(0, state_to_add.__len__()):
            try:
                if state_to_add[index] not in bts.nodes():
                    bts.add_node(state_to_add[index])
                bts.add_edge(edge_to_add[index][0], edge_to_add[index][1])
            except:
                pass

        # 下一步
        # 验证UR的求解
        # 同时计算Y state
        # 如果没有Y State则停止迭代


    return bts

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

    bts = t_aic_onestep(iwa, ['0', '6'], event_uo, event_o, event_c, event_uc)

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
    nx.draw(bts, pos=nx.spring_layout(bts), with_labels=True, font_size=6.5) # https://www.jianshu.com/p/e254cd6acfdc/
                                                                          # https://blog.csdn.net/HsinglukLiu/article/details/107821649

    plt.show()

if __name__ == '__main__':
    main()
    print('Finished')
