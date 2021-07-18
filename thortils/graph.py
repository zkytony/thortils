### CODE copied from zkytony/graphspn repository.

import random
from abc import ABC, abstractmethod
from collections.abc import Iterable
import copy
from collections import deque

########################################
#  Node
########################################
class Node:

    def __init__(self, id):
        """
        The id is expected to be unique in the graph.
        """
        self.id = id

    @property
    def coords(self):
        """returns an (x,y) location on the plane, for visualization purposes.
        The coordinates have resolution """
        if hasattr(self, "_coords"):
            return self._coords
        else:
            self._coords = (random.randint(-500, 500),
                            random.randint(-500, 500))
            return self._coords

    def __repr__(self):
        return "%s(%d)" % (type(self).__name__, self.id)


class SuperNode(Node, ABC):

    def __init__(self, id, enset):
        """
        id (int) is for this node
        enset (OrderedEdgeNodeSet) underlying graph structure
        """
        self.id = id
        if type(enset) != OrderedEdgeNodeSet:
            raise ValueError("super node must use OrderedEdgeNodeSet!")
        self.enset = enset

    @classmethod
    @abstractmethod
    def pick_id(cls, enset, existing_ids):
        """
        that always returns the same id if given the same subgraph, and different
        if otherwise.
        """
        pass

    def nodes_list(self):
        return self.enset.nodes_list

########################################
#  Edge
########################################
class Edge:
    """
    An edge links two nodes.
    """

    def __init__(self, id, node1, node2, data=None):
        """
        The id is expected to be unique in the graph edges.
        """
        self.id = id
        if node2 is None:
            self.nodes = (node1,)
        else:
            self.nodes = (node1, node2)

        self.data = data
        # # view numbers
        # if view_nums is None:
        #     self.view_nums = (util.compute_view_number(node1, node2),
        #                       util.compute_view_number(node2, node1))
        # else:
        #     assert len(view_nums) == 2
        #     self.view_nums = view_nums

    @property
    def degenerate(self):
        return len(self.nodes) == 1

    def __repr__(self):
        if self.data is None:
            data = "--"
        else:
            data = self.data
        if not self.degenerate:
            return "#%d[<%d>%s<%d>]" % (self.id, self.nodes[0].id, str(data), self.nodes[1].id)
        else:
            return "#%d[<%d>]" % (self.id, self.nodes[0].id)


class SuperEdge(Edge, ABC):

    def __init__(self, id, supernode1, supernode2, data=None):
        """
        id (int) id for this edge
        supernode1 (SuperNode) a super node
        supernode2 (SuperNode) a super node
        """
        super().__init__(id, supernode1, supernode2, data=data)

    @classmethod
    @abstractmethod
    def pick_id(self, supernode1, supernode2, edge=None, existing_ids=set({})):
        """
        function that always returns the same id if given the
        same subgraphs on two ends and an edge on the original graph,
        and different if otherwise.
        """
        pass



class EdgeNodeSet:
    """
    EdgeNodeSet is simply a tuple (V, E) but without restriction on whether the
    edges and vertices have any correspondence. It may be a graph, or not.
    """
    def __init__(self, nodes, edges):
        """
        edges (set or dict) map from edge id to an edge object
        nodes (set or dict) map from node id to a node object

        Both could be None.
        """
        if nodes is None:
            nodes = {}
        if edges is None:
            edges = {}
        if type(nodes) == set:
            nodes = {e.id:e for e in nodes}
        if type(edges) == set:
            edges = {e.id:e for e in edges}

        self._nodes = nodes
        self._edges = edges

    @property
    def nodes(self):
        return self._nodes

    @property
    def edges(self):
        return self._edges

    def num_nodes(self):
        return len(self._nodes)

    def num_edges(self):
        return len(self._edges)

    def _build_degen_edges(self):
        # Given that edges is empty, returns a set of degenerate edges, each
        # attached to a node
        assert len(self._edges) == 0
        edges = set({})
        for nid in self._nodes:
            edges.add(Edge(nid, self._nodes[nid]))
        return edges

    def to_graph(self, directed=False):
        # Verify integrity
        for eid in self._edges:
            nodes = self._edges[eid].nodes
            if nodes[0].id not in self._nodes:
                return None
            if nodes[1].id not in self._nodes:
                return None
        return Graph(self._edges, directed=directed)

    def to_unused_graph(self, directed=False, **params):
        edges = self._edges
        return UnusedGraph(edges, **params)

# absent_node_ids (set): nodes in this set are actually "not present", meaning
#     they provide the structure for this graph, but they are excluded
#     in function calls, such as neighbors() and partition().

class OrderedEdgeNodeSet(EdgeNodeSet):

    def __init__(self, nodes, edges):
        """
        edges (list) list of edge objects
        nodes (list) list of node objects

        Both could be None.
        """
        super().__init__(set(nodes), set(edges))
        self._ordered_nodes = nodes
        self._ordered_edges = edges

    @property
    def nodes_list(self):
        return self._ordered_nodes

    @property
    def edges_list(self):
        return self._ordered_edges


########################################
#  Graph
########################################
class Graph(EdgeNodeSet):

    def __init__(self, edges, directed=False):
        """
        edges (set or dict) map from edge id to an edge object
        A graph could be a simple/multi-graph and a (un)directed graph.
        """
        # Build nodes map
        nodes = {}
        if type(edges) == set:
            for edge in edges:
                endpoints = edge.nodes
                if endpoints[0].id not in nodes:
                    nodes[endpoints[0].id] = endpoints[0]
                if edge.degenerate: # degenerate edge; only one node.
                    continue
                if endpoints[1].id not in nodes:
                    nodes[endpoints[1].id] = endpoints[1]
            edges = {e.id:e for e in edges}
        elif type(edges) == dict:
            for eid in edges:
                endpoints = edges[eid].nodes
                if endpoints[0].id not in nodes:
                    nodes[endpoints[0].id] = endpoints[0]
                if edges[eid].degenerate:  # degenerate edge; only one node.
                    continue
                if endpoints[1].id not in nodes:
                    nodes[endpoints[1].id] = endpoints[1]

        super().__init__(nodes, edges)
        self._directed = directed

        # keep track of connections for faster graph operations
        self._conns = {nid : {} for nid in self._nodes}
        self._outedges = {nid : set({}) for nid in self._nodes}
        self._build_connections()


    @property
    def directed(self):
        return self._directed

    def is_empty(self):
        return self.num_edges() == 0

    # @abstractmethod
    # def _build_connections(self):
    def _build_connections(self):
        """Builds the self._conns field, which is a map from node id to a dictionary of neighbor id -> edge(s).
        Implementation differs between different types of graphs."""
        self._multi = False
        for eid in self._edges:
            if self._edges[eid].degenerate:  # degenerate, only one node.
                node1 = self._edges[eid].nodes[0]
            else:
                node1, node2 = self._edges[eid].nodes
            if node1.id not in self._conns:
                self._conns[node1.id] = {}
                self._outedges[node1.id] = set({})
            if self._edges[eid].degenerate:  # degenerate, only one node.
                continue
            if node2.id not in self._conns[node1.id]:
                self._conns[node1.id][node2.id] = set({})
            self._conns[node1.id][node2.id].add(eid)
            self._outedges[node1.id].add(eid)
            if len(self._conns[node1.id][node2.id]) > 1:
                self._multi = True
            if not self._directed:
                if node2.id not in self._conns:
                    self._conns[node2.id] = {}
                if node1.id not in self._conns[node2.id]:
                    self._conns[node2.id][node1.id] = set({})
                self._conns[node2.id][node1.id].add(eid)
                self._outedges[node2.id].add(eid)

    #--- Basic graph operations ---#
    def is_neighbor(self, node_id, test_id):
        return test_id in self._conns[node_id]

    def neighbors(self, node_id):
        """
        Returns a set of neighbor node ids
        """
        return set(self._conns[node_id].keys())

    def edges_between(self, node1_id, node2_id):
        """Return edge id(s) between node 1 and node 2; The returned object depends on
        the child class's implementation of _build_connections()"""
        if node2_id not in self._conns[node1_id]:
            return None
        else:
            return self._conns[node1_id][node2_id]# {self.edges[eid] for eid in }

    def edges_from(self, node_id):
        """Return edge id(s) from node_id"""
        return self._outedges[node_id]


    def copy(self):
        """
        Returns a new Graph which contains the same information as `self`. The new object
        is completely separate from `self`, meaning modifying any information in the copied topo-map
        does not affect `self`.
        """
        edges_copy = copy.deepcopy(self._edges)
        return self.__class__(edges_copy)

    def connected_components(self):
        """
        Returns the connected components in this graph, each as a separate TopologicalMap instance.

        Note: The union of the sets of node ids in the returned connected components equals
        to the original topo map's set of node ids. (i.e. node ids are kept the same in components)
        """
        # Uses BFS to find connected components
        copy_graph = self.copy()
        to_cover = set(copy_graph.nodes.keys())
        components = []
        while len(to_cover) > 0:
            start_nid = random.sample(to_cover, 1)[0]
            q = deque()
            q.append(start_nid)
            component_edge_ids = set({})
            visited = set()
            while len(q) > 0:
                nid = q.popleft()
                neighbors = copy_graph.neighbors(nid)
                for neighbor_nid in neighbors:

                    eid = copy_graph.edges_between(nid, neighbor_nid)
                    if isinstance(eid, Iterable):  # multi-graph
                        for e in eid:
                            component_edge_ids.add(e)
                    else:
                        component_edge_ids.add(eid)

                    if neighbor_nid not in visited:
                        visited.add(neighbor_nid)
                        q.append(neighbor_nid)
            # build component
            component_edges = {eid : copy_graph.edges[eid] for eid in component_edge_ids}
            component = self.__class__(component_edges)
            components.append(component)
            to_cover -= set(component.nodes.keys())
        return components

    def subtract(self, other):
        """
        Given another graph "other", produce a graph equivalent
        as subtracting the `other` from this graph. It does not
        matter whether other_map contains nodes that are not in this graph.
        """
        nodes = {}
        edges = {}
        for eid in self.edges:
            if eid not in other.edges:
                edges[eid] = copy.deepcopy(self.edges[eid])
        return self.__class__(edges)


    def partition_by_templates(self, templates,
                               super_node_class=SuperNode, super_edge_class=SuperEdge, **params):
        """
        Invariant: The total number of underlying nodes and edges (i.e. variables) in the result
        is always equal to that in the original graph.
        """
        current_graph = self
        results = {}

        for template in templates:
            print(template.__name__)
            unused_graph, super_graph = current_graph.partition(template, get_unused=True,
                                                                super_node_class=super_node_class,
                                                                super_edge_class=super_edge_class, **params)
            current_graph = unused_graph
            results[template.__name__] = super_graph
        # assert that unused graph is empty and super graph contains the same number of underlying nodes and edges
        count = sum(results[template.__name__].num_nodes() * template.size()
                    for template in templates)
        assert count == self.num_nodes() or (count == self.num_nodes() + self.num_edges())
        return results, count

    def _partition_setup(self):
        edges_available = set(self.edges.keys())
        edges_used = set({})
        nodes_used = set({})  # nodes already used to create new graph
        nodes_available = set(self.nodes.keys()) - nodes_used
        return edges_available, edges_used, nodes_available, nodes_used

    def partition(self, template, get_unused=False,
                  super_node_class=SuperNode, super_edge_class=SuperEdge, **params):
        """
        Partitions this graph by a given template. This algorithm is a realization of the
        Algorithm 2 in http://kaiyuzheng.me/documents/papers/zheng2018aaai.pdf, more
        generalized than Algorithm 1 in http://kaiyuzheng.me/documents/papers/zheng2017thesis.pdf.

        The given template can be of the following types:
        - Only nodes
        - Nodes and edges
        We choose not to support templates with only edges

        Invariant: One edge or one node can only belong to one template (i.e. super node).


        Args:

          template (Template): a Template class. Note that a template is not necessarily a graph.
                               It is generally an EdgeNodeSet, because a valid template can have,
                               for example, only edges and no nodes.
          get_unused (bool): If 'get_unused' is true, return a tuple where the first element
                             is the 'supergraph' and the second element is a graph where nodes
                             are not covered by this partition attempt.
          super_node_class (SuperNode): a SuperNode class that represents
                             the subgraph formed by multiple nodes in the original graph.
          super_edge_class (SuperEdge): a SuperEdge class is an edge added when building the
                             supergraph; it contains functionality to compute the data field
                             for this edge based on the two super nodes on its ends.

        Note that the supergraph will always not contain data on edges, since these data
        may have been covered by some template, which is consolidated into a super node.

        Returns:
          a new Graph that is a 'supergraph' based on this one, or a tuple ('supergraph', 'unused_graph')

        """
        random.seed()
        edges_available, edges_used, nodes_available, nodes_used = self._partition_setup()

        spnodes = {}  # sp: super
        spedges = {}
        spconns = {}

        node_sn = {}  # map from nid to snid
        edge_sn = {}  # map from eid to snid

        while len(edges_available) > 0:
            # randomly sample an edge, then a node on that edge, as the starting point of template matching.
            eid = random.sample(edges_available, 1)[0]
            vindx = random.sample([0,1], 1)[0]
            if len(self.edges[eid].nodes) == 1:
                import pdb; pdb.set_trace()
            nid = self.edges[eid].nodes[vindx].id
            enset = None
            if nid in nodes_used:
                vindx = 1 - vindx
                nid = self.edges[eid].nodes[vindx].id
                if nid in nodes_used:
                    if template.num_nodes() > 0:
                        # This edge is not useful to match this template because both nodes are not present
                        edges_available.remove(eid)
                        continue
                    else:
                        # The given template has no nodes. So it's an edge template. Pass in the sampled edge.
                        enset = template.match(self, None, self.edges[eid],
                                               nodes_used, edges_used, **params)
                        if enset is None:
                            edges_available.remove(eid)
                            continue  # try another edge. It's no use to keep matching on this one.
            if enset is None:
                # The given template can be tried at the sampled node (nid). Try to match it.
                enset = template.match(self, self.nodes[nid], self.edges[eid],
                                       nodes_used, edges_used, **params)
            if enset is None:
                # Unable to match.
                edges_available.remove(eid)
            else:
                # Matched!
                edges_available -= set(enset.edges.keys())
                edges_used |= set(enset.edges.keys())
                nodes_available -= set(enset.nodes.keys())
                nodes_used |= set(enset.nodes.keys())

                # super node
                snid = super_node_class.pick_id(enset, spnodes)
                if snid not in spnodes:
                    spnodes[snid] = super_node_class(snid, enset)
                    spconns[snid] = {}

                # connectivity of super nodes
                for nid in enset.nodes:
                    # check neighbors. If neighbor belongs to a super node, connect the two.
                    for nnid in self.neighbors(nid):
                        if nnid in node_sn:
                            # proof: a proof that the two super nodes are connected. ALWAYS an edge id.
                            #        this is used to deal with multigraphs and mixture of template types.
                            #        See figure for more details.
                            proof = random.sample(self.edges_between(nid, nnid), 1)[0]
                            seid = super_edge_class.pick_id(spnodes[snid], spnodes[node_sn[nnid]],
                                                            existing_ids=set(spedges.keys()))
                            spedges[seid] = super_edge_class(seid, spnodes[snid], spnodes[node_sn[nnid]])
                            if node_sn[nnid] not in spconns[snid]:
                                spconns[snid][node_sn[nnid]] = set({})
                            spconns[snid][node_sn[nnid]].add(proof)
                    node_sn[nid] = snid

                for eid in enset.edges:
                    # check edges outgoing from both ends. If an edge belong to a super node, connect the two.
                    for node in enset.edges[eid].nodes:
                        for eid in self.edges_from(node.id):
                            if eid in edge_sn and edge_sn[eid] in spconns[snid]:
                                # If this edge is not already used as a proof
                                if eid not in spconns[snid][edge_sn[eid]]:
                                    proof = eid
                                    seid = super_edge_class.pick_id(spnodes[snid], spnodes[edge_sn[eid]],
                                                                    existing_ids=set(spedges.keys()))
                                    spedges[seid] = super_edge_class(seid, spnodes[snid], spnodes[edge_sn[eid]])
                                    if edge_sn[eid] not in spconns[snid]:
                                        spconns[snid][edge_sn[eid]] = set({})
                                    spconns[snid][edge_sn[eid]].add(proof)
                    edge_sn[eid] = snid

        # Build the "edges" set for the super graph. It includes spedges, and also degenerate edges for
        # super nodes without connectivity.
        for snid in spnodes:
            if len(spconns[snid]) == 0:
                # No connectivity. Degenerate edge
                seid = super_edge_class.pick_id(spnodes[snid], None,
                                existing_ids=set(spedges.keys()))
                spedges[seid] = super_edge_class(seid, spnodes[snid], None)

        supergraph = Graph(spedges, directed=self._directed)

        if get_unused:
            unused_nodes = {nid: self.nodes[nid] for nid in self.nodes
                            if nid not in nodes_used}
            unused_edges = {eid: self.edges[eid] for eid in self.edges
                            if eid not in edges_used}
            unused_graph = EdgeNodeSet(unused_nodes, unused_edges).to_unused_graph(directed=self._directed,
                                                                                   unused_node_ids=set(unused_nodes.keys()),
                                                                                   unused_edge_ids=set(unused_edges.keys()))
            return unused_graph, supergraph
        else:
            return supergraph


    # #--- Visualizations ---#
    # def visualize(self, ax, included_nodes=None, dotsize=10, linewidth=1.0,
    #               img=None, show_nids=False,
    #               **params):  # params = canonical_map_yaml_path=None, consider_placeholders=False,
    #     """Visualize the topological map `self`. Nodes are colored by labels, if possible.
    #     If `consider_placeholders` is True, then all placeholders will be colored grey.
    #     Note that the graph itself may or may not contain placeholders and `consider_placholders`
    #     is not used to specify that."""

    #     # Plot the nodes
    #     for nid in self.nodes:
    #         if included_nodes is not None and nid not in included_nodes:
    #             continue

    #         nid_text = str(nid) if show_nids else None

    #         node = self.nodes[nid]
    #         node_color = node.color
    #         x, y = node.coords  # gmapping coordinates
    #         plot_x, plot_y = util.plot_dot(ax, x, y,
    #                                        dotsize=dotsize, color=node_color, zorder=2,
    #                                        linewidth=linewidth, edgecolor='black', label_text=nid_text, alpha=0.6)

    #         # Plot the edges
    #         for neighbor_id in self._conns[nid]:
    #             if included_nodes is not None and neighbor_id not in included_nodes:
    #                 continue

    #             edges = self.edges_between(nid, neighbor_id)
    #             for i, eid in enumerate(edges):
    #                 edge = self._edges[eid]
    #                 util.plot_line(ax, node.coords, self.nodes[neighbor_id].coords,
    #                                linewidth=3, color=edge.color, zorder=1, alpha=0.2)



class UnusedGraph(Graph):
    """
    This is specifically used to deal with "leftover" graphs in after partition.
    This graph may contain nodes and edges that are present for structure but cannot
    actually be used for template matching any more.
    """

    def __init__(self, edges, unused_node_ids=set({}), unused_edge_ids=set({}), directed=False):
        super().__init__(edges, directed=directed)
        self._uu_nids = unused_node_ids
        self._uu_eids = unused_edge_ids

    def _partition_setup(self):
        edges_available = self._uu_eids
        edges_used = set(self.edges.keys()) - set(self._uu_eids)
        nodes_available = self._uu_nids
        nodes_used = set(self.nodes.keys()) - set(self._uu_nids)
        return edges_available, edges_used, nodes_available, nodes_used

    def neighbors(self, node_id):
        """
        Returns a set of neighbor node ids
        """
        return set({nid
                    for nid in set(self._conns[node_id].keys())
                    if nid in self._uu_nids})

    def num_nodes(self):
        return len(self._uu_nids)

    def num_edges(self):
        return len(self._uu_eids)
