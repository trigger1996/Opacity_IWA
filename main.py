# coding=UTF-8
import networkx as nx
import io
import yaml
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

def dfs_aic(dfs_tree, sc, source=None, depth_limit=None):
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
                    #if event_t == dfs_tree.edges[parent, child, 0]['event'] and dfs_tree.edges[parent, child, 0]['t_min'] <= t \
                    #   and dfs_tree.edges[parent, child, 0]['t_max'] > t:                                                       # 在aic的dfs搜索过程中对t_min的判断还是要有的，不信删掉走一下就会发现结果很离谱
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
    Y_state = bts_start
    for y_node in Y_state:

        curr_node = y_node                          # 预留这个，方便迭代

        for sc in supervisior_ut:
            dfs_tree = dfs_events(iwa, sc, curr_node)
            t_interval = timeslice(dfs_tree)

            # 求出不同事件对应的时间的最大值
            max_time_uo = {}
            for event_t in list(sc):
                max_time_uo.update({event_t: -1})
            for u, v, data in dfs_tree.out_edges(data=True):                 # u, v, data in dfs_tree.out_edges(curr_node, data=True)
                if data['event'] in sc and data['t_max'] > max_time_uo[event_t]:
                        max_time_uo[data['event']] = data['t_max']


            # 从timeslice内选取不超过时间最大值的时间，给对应事件
            sc_event_tuple = []
            for _iter in max_time_uo.keys():
                sc_timed_t = []
                for t in t_interval:
                    if t <= max_time_uo[_iter]:
                        sc_timed_t.append((_iter, t))
                if sc_timed_t.__len__() != 0:
                    sc_event_tuple.append(sc_timed_t)       # 加入非空元素

            # 所有可能对应的时间，去各取一个
            # {'a': [1, 2, 3, 4, 5, 6, 7, 9], 'b': [1, 2, 3, 4, 5, 6, 7], 'o3': [1, 2, 3, 4, 5, 6], 'uc': []}
            # 取出(a, 1), (b, 2), (o3, 1)
            sc_set = list(product(*sc_event_tuple))                 # https://blog.csdn.net/weixin_39652760/article/details/110774080
                                                                    # https://blog.csdn.net/liuqiang3/article/details/99707294
            sc_set = [list(_iter) for _iter in sc_set]              # tuple -> list
                                                                    # 而且最后拿到的数据是排序排好的，这里就不用考虑连接问题


            for supervisior_curr in sc_set:

                last_state = Y_state

                # dfs遍历生成的树
                # 起点：当前状态所有的点
                # 目标：找到所有的可达点
                try:
                    reachable_edge = list(dfs_aic(dfs_tree, supervisior_curr, source=curr_node))
                    #print(supervisior_curr, reachable_edge)

                    # edge -> 可达点
                    ur = []
                    ur_new = []
                    ur.append(curr_node)
                    for edge_t in reachable_edge:
                        ur.append(list(edge_t)[1])          # 可以发生transition的终点都是可达的点

                    z_state = (tuple(ur), tuple(supervisior_curr))
                    if z_state not in bts.nodes():   ##
                        bts.add_node(z_state)
                    bts.add_edge(last_state, z_state)
                    last_state = z_state                    # 迭代更新，因为前面edge都是排好的，所以这里直接加进来

                except:
                    pass

        # 这段代码没啥用
        maxIndex = 1  # Here
        for u, v, data in dfs_tree.in_edges(curr_node, data=True):        # https://stackoverflow.com/questions/37230751/networkx-get-all-in-edges-of-a-node
            print(u, v, data)

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

    bts = t_aic_onestep(iwa, '0', event_uo, event_o, event_c, event_uc)

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
    nx.draw(bts)

    plt.show()

if __name__ == '__main__':
    main()
    print('Finished')
