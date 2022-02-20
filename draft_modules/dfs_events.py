import networkx as nx
import io
import yaml
import matplotlib.pyplot as plt # 导入 Matplotlib 工具包

# https://blog.csdn.net/u010330109/article/details/89525729
# 它这个思路是求所有不连通的子图，那么可以吧\Sigma_o当成断开的进行求解即可，求完加上去

max_edge_number = 3

event_uo = ['a', 'b', 'uc']

def dfs_edges(G, event_list, source=None, depth_limit=None):
    """Iterate over edges in a depth-first-search (DFS).

    Perform a depth-first-search over the nodes of G and yield
    the edges in order. This may not generate all edges in G (see edge_dfs).

    Parameters
    ----------
    G : NetworkX graph

    source : node, optional
       Specify starting node for depth-first search and return edges in
       the component reachable from source.

    depth_limit : int, optional (default=len(G))
       Specify the maximum search depth.

    Returns
    -------
    edges: generator
       A generator of edges in the depth-first-search.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> list(nx.dfs_edges(G, source=0))
    [(0, 1), (1, 2), (2, 3), (3, 4)]
    >>> list(nx.dfs_edges(G, source=0, depth_limit=2))
    [(0, 1), (1, 2)]

    Notes
    -----
    If a source is not specified then a source is chosen arbitrarily and
    repeatedly until all components in the graph are searched.

    The implementation of this function is adapted from David Eppstein's
    depth-first search function in `PADS`_, with modifications
    to allow depth limits based on the Wikipedia article
    "`Depth-limited search`_".

    .. _PADS: http://www.ics.uci.edu/~eppstein/PADS
    .. _Depth-limited search: https://en.wikipedia.org/wiki/Depth-limited_search

    See Also
    --------
    dfs_preorder_nodes
    dfs_postorder_nodes
    dfs_labeled_edges
    edge_dfs
    bfs_edges
    """
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
                if str([parent, child]) not in visited:     # 因为list本身不可哈希，所以用str(list())来代替list
                    yield parent, child                     # yield parent, child 这个版本的python没法调试yield  https://zhuanlan.zhihu.com/p/268605982

                    visited.add(str([parent, child]))       # visited.add(child)
                    if depth_now > 1:
                        try:
                            for index in range(0, max_edge_number):
                                if G.edges[parent, child, index]['event'] in event_list:
                                    stack.append((child, depth_now - 1, iter(G[child])))
                                    #print(G.edges['0', '1',index])
                        except:
                            pass



            except StopIteration:
                stack.pop()

def dfs_events(iwa, event_list, source):
    edges = list(dfs_edges(iwa, event_list, source))

    print(edges)

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

            if event not in event_list:
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

    #result = list(dfs_edges(iwa, event_uo, source='0'))         # nx.dfs_tree nx.dfs_predecessors
    #print(result)

    dfs_tree = dfs_events(iwa, event_uo, source='8')

    '''
        Plotting
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
        print(edge_t)
        min_t = dfs_tree.edges[list(edge_t)[0], list(edge_t)[1], 0]['t_min']
        max_t = dfs_tree.edges[list(edge_t)[0], list(edge_t)[1], 0]['t_max']
        labels.update({edge_t : str('[' + str(min_t) + ', ' + str(max_t) + ']')})
    nx.draw_networkx_edge_labels(dfs_tree, pos, edge_labels=labels, font_color="c")  # 显示权值

    plt.show()

if __name__ == '__main__':
    main()
    print('Finished')
