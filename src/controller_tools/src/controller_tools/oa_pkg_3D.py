import numpy as np
from collections import deque, defaultdict
from time import perf_counter


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class Circle():
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def points_in_circles(self, points):
        dimension = 3  # 3D
        nb_of_circles = len(self.center)
        nb_of_points = len(points)
        Cj = np.tile(self.center, nb_of_points).reshape(
            (nb_of_circles, nb_of_points, dimension))
        result = Cj-points
        # result = np.linalg.norm(result, axis=-1) <= self.radius
        result = result**2
        result = np.sum(result, axis=-1)
        result = result <= self.radius**2
        result = np.any(result, axis=0)
        return result


class Segment():
    def __init__(self, A, B):
        self.directional_vector = B-A
        # self.length_sq = (self.directional_vector[0]**2+self.directional_vector[1]**2+self.directional_vector[2]**2)
        self.length_sq = np.sum(self.directional_vector**2, axis=-1)
        self.length = self.length_sq**0.5
        self.A = A
        self.B = B

    def is_intersecting_circles(self, circles):
        CA = self.A-circles.center
        CA_sq = CA**2
        dot_prod = CA*self.directional_vector
        # dot_prod = dot_prod[:, 0]+dot_prod[:, 1]+dot_prod[:, 2]
        # norm_CA_sq = CA_sq[:, 0]+CA_sq[:, 1]+CA_sq[:, 2]
        dot_prod = np.sum(dot_prod, axis=-1)
        norm_CA_sq = np.sum(CA_sq, axis=-1)
        c = norm_CA_sq-circles.radius**2
        b = 2*dot_prod
        a = self.length_sq
        delta = b**2-4*a*c
        pos_deltas = delta >= 0
        is_any_delta_pos = pos_deltas.any()
        if not is_any_delta_pos:
            return False
        else:
            b = b[pos_deltas]
            c = c[pos_deltas]
            delta = delta[pos_deltas]
            a1 = -b/(2*a)
            a2 = np.sqrt(delta)/(2*a)
            root_1 = a1-a2
            # root_2 = a1+a2
            # isIntersecting=((root_1 >= 0)&(root_1 <= 1)).any() or ((root_2 >= 0)&(root_2 <= 1)).any()
            if ((root_1 <= 1) & (root_1 >= 0)).any():
                isIntersecting = True
            else:
                root_2 = a1+a2
                if ((root_2 <= 1) & (root_2 >= 0)).any():
                    isIntersecting = True
                else:
                    isIntersecting = False
        return isIntersecting


class Segments():
    def __init__(self, A, B):
        self.points = A  # path must be a line matrix
        self.directional_vectors = B-A
        self.lengths_sq = self.directional_vectors[:,
                                                   0]**2+self.directional_vectors[:, 1]**2
        self.lengths = self.lengths_sq**0.5  # li

    def intersecting_circles(self, circles):
        dimension = 3  # 3D
        n = len(circles.center)
        m = len(self.points)
        Ai = np.tile(self.points, n).reshape(
            (m, n, dimension)).transpose((1, 0, 2))
        Cj = np.tile(circles.center, m).reshape((n, m, dimension))
        AiCj = Cj-Ai
        AiCj_sq = AiCj**2
        # norm_AiCj_sq = AiCj_sq[:, :, 0]+AiCj_sq[:, :, 1]
        norm_AiCj_sq = np.sum(AiCj_sq, axis=-1)
        a = self.lengths_sq
        b = -2*np.sum(AiCj*self.directional_vectors, axis=-1)
        c = (norm_AiCj_sq.T-circles.radius**2).T
        delta = b**2-4*a*c
        pos_deltas = delta >= 0
        is_delta_neg = (np.logical_not(pos_deltas))
        is_delta_neg = np.any(is_delta_neg, axis=0)

        # If all deltas are positive
        if not is_delta_neg.any():
            isIntersecting = np.ones(m, dtype=bool) & False
        else:
            pos_deltas = np.where(pos_deltas)
            b = b[pos_deltas]
            c = c[pos_deltas]
            a = a[pos_deltas[1]]
            delta = delta[pos_deltas]
            a1 = -b/(2*a)
            a2 = np.sqrt(delta)/(2*a)
            root_1 = a1-a2
            root_2 = a1+a2
            # Testing the roots
            root_1 = (0 <= root_1) & (root_1 <= 1)
            root_2 = (0 <= root_2) & (root_2 <= 1)
            isIntersecting = root_1 | root_2
            prb_sgts = np.ones(m, dtype=bool) & False
            prb_idx = pos_deltas[1][isIntersecting]
            prb_sgts[prb_idx] = True
            isIntersecting = prb_sgts
        return isIntersecting


class Line():
    def __init__(self, A, B):
        self.directional_vector = (A-B)/np.linalg.norm(A-B)
        self.A = A
        self.B = B

    def is_intersecting_circle(self, circle):
        r2 = circle.center-self.A
        norm_sq_r2 = np.dot(r2, r2)
        dx = np.dot(r2, self.directional_vector)
        isIntersecting = (norm_sq_r2-dx**2) < circle.radius**2
        return isIntersecting

    def is_intersecting_circles(self, circles):
        A_to_center = circles.center-self.A
        norm_sq_r2 = A_to_center**2
        norm_sq_r2 = norm_sq_r2[:, 0]+norm_sq_r2[:, 1]
        dx = A_to_center*self.directional_vector
        dx = dx[:, 0]+dx[:, 1]
        isIntersecting = (norm_sq_r2-dx**2) < circles.radius**2
        return isIntersecting.any()


class Tree():
    def __init__(self, start, end, obstacles, safety_radius=1):
        self.edges = defaultdict(lambda: deque())
        self.nodes = deque()
        self.costs = defaultdict(lambda: 0)
        self.add_node(start)
        self.start = np.array(start)
        self.end = np.array(end)
        self.se = self.end-self.start
        self.obstacles = obstacles
        self.circles = Circle(
            self.obstacles, safety_radius*np.ones(len(self.obstacles)))
        self.safety_radius = safety_radius
        self.nb_of_iterations = 5000
        self.dimension = 3

    def add_edge(self, node_a, nodes):
        self.add_node(node_a)
        if isinstance(nodes, list):
            for node in nodes:
                if node not in self.edges[node_a]:
                    self.edges[node_a].append(node)
                    self.edges[node].append(node_a)
                    self.add_node(node)
        else:
            if nodes not in self.edges[node_a]:
                self.edges[node_a].append(nodes)
                self.edges[nodes].append(node_a)
                self.add_node(nodes)

    def add_node(self, node):
        if node not in self.nodes:
            self.nodes.append(node)

    def add_nodes(self, nodes):
        set_nodes = set(nodes)
        common_elements = set_nodes.intersection(self.nodes)
        set_nodes.difference(common_elements)
        self.nodes.extend(set_nodes)

    def rrt(self, search_space=[-10, 10, -10, 10, -5, 5]):
        nb_of_iterations = 5000
        # rnd_points = self.random_points([-10, 10, -10, 10], nb_of_iterations)
        rnd_points = self.random_points(search_space, nb_of_iterations)

        t0 = perf_counter()
        success = False
        for k in range(nb_of_iterations):
            rnd_point = rnd_points[k]
            # rnd_point=self.random_point()
            # Link the new node to the nearest node in the tree if there's no collision
            iters = k
            if self.extend(rnd_point) == 'reached':
                print('found the way', 'number of iters:', iters)
                success = True
                break
        t1 = 1000*(perf_counter()-t0)
        print('rrt time: ', t1, ' ms')
        if not success:
            print(bcolors.WARNING +
                  '[WARN] Local planner failed to find a path. Cancelling path.'+bcolors.ENDC)
        return success

    def extend(self, node):
        # Find the nearest neighbor
        # If the line between node and the nearest neighbor doesn't intersect, connect the nodes
        # Else: go to the next iteration

        # Nearest node
        graph_nodes = np.array(self.nodes)
        distances = np.linalg.norm(graph_nodes-node, axis=-1)
        p = np.argmin(distances)
        nearest_node = graph_nodes[p]
        dist = distances[p]
        step = 0.5
        d = np.min([dist, step])
        node = nearest_node+(node-nearest_node)/dist*d
        line = Segment(node, nearest_node)
        result = line.is_intersecting_circles(self.circles)
        if result:
            return 'trapped'
        else:
            if (node != self.end).any():
                line = Segment(node, self.end)
                result = line.is_intersecting_circles(self.circles)
                if not result:
                    self.add_edge(tuple(nearest_node), tuple(node))
                    self.add_edge(tuple(node), tuple(self.end))
                    return 'reached'
        eps = 0.25
        t_nearest_node = tuple(nearest_node)
        t_node = tuple(node)
        self.add_edge(t_nearest_node, t_node)
        self.costs[t_node] = self.costs[t_nearest_node]+d
        if np.linalg.norm(node-self.end) < eps:
            t_end = tuple(self.end)
            if (self.end != node).any():
                self.costs[t_end] = self.costs[t_node] + \
                    np.linalg.norm(self.end-node)
                self.add_edge(t_node, t_end)
            return 'reached'
        return 'continue'

    def random_points(self, search_area, nb_of_iterations):
        dimension = self.dimension
        Xmin, Xmax = search_area[::2], search_area[1::2]
        Xmin, Xmax = np.array(Xmin), np.array(Xmax)
        points = np.random.rand(nb_of_iterations, dimension)
        points = Xmin + (Xmax-Xmin)*points
        goal_probability = 0.25
        probs = np.random.rand(nb_of_iterations)
        points[probs < goal_probability] = self.end
        return points

    def dijkstra(self):
        srcIdx = tuple(self.start)
        dstIdx = tuple(self.end)
        if dstIdx not in self.nodes:
            print('there is no path joining the start and the end')
            return []
        # build dijkstra
        nodes = list(self.edges.keys())
        dist = {node: float('inf') for node in nodes}
        prev = {node: None for node in nodes}
        dist[srcIdx] = 0
        while nodes:
            curNode = min(nodes, key=lambda node: dist[node])
            nodes.remove(curNode)
            if dist[curNode] == float('inf'):
                break

            for neighbor in self.edges[curNode]:
                # cost=np.linalg.norm(np.array(neighbor)-np.array(curNode))
                cost = self.costs[neighbor]
                newCost = dist[curNode] + cost
                if newCost < dist[neighbor]:
                    dist[neighbor] = newCost
                    prev[neighbor] = curNode

        # retrieve path
        path = deque()
        curNode = dstIdx
        while prev[curNode] is not None:
            path.appendleft(curNode)
            curNode = prev[curNode]
        path.appendleft(curNode)
        self.path = np.array(path)
        return self.path

    def add_middle_points(self, points):
        n = len(points)
        dimension = self.dimension
        if n < 3:
            return points
        new_points = np.zeros((2*n-1, dimension))
        new_points[0] = points[0]
        new_points[2::2] = points[1:]
        new_points[1::2] = (points[1:]+points[:-1])/2
        return new_points

    def add_random_points(self, points):
        n = len(points)
        new_points = np.zeros((2*n-1, 2))
        new_points[0] = points[0]
        new_points[2::2] = points[1:]
        # Center around the middle with a span of +/- 0.25
        r = (0.5+(np.random.rand(n-1, 1)-0.5)*0.25)
        dir = points[1:]-points[:-1]
        new_points[1::2] = points[:-1]+dir*r
        return new_points

    def shorten_path(self, path):
        new_path = deque()
        A = path[:-2]
        B = path[2:]
        segments = Segments(A, B)
        bad_segments = segments.intersecting_circles(self.circles)
        i = 0
        A = path[i]
        new_path.append(A)
        while True:
            if i >= len(path)-2:
                if i == len(path)-2:
                    new_path.append(path[-1])
                break
            A = path[i]
            B = path[i+1]
            C = path[i+2]
            AC = Segment(A, C)
            AB = Segment(A, B)
            BC = Segment(B, C)
            okay = not (bad_segments[i])
            if okay and AC.length < AB.length+BC.length:
                new_path.append(C)
                i += 2
            else:
                new_path.append(B)
                i += 1
        new_path = np.array(new_path)
        return new_path

    def smoothen_path(self, new_path, iterations=7):
        self.lenghts = []
        new_path = np.array(new_path)
        for i in range(iterations):
            new_path = self.add_middle_points(new_path)
            for _ in range(6):
                new_path = self.shorten_path(new_path)
            # self.lenghts.append(np.sum(np.linalg.norm(
            #     new_path[1:]-new_path[:-1], axis=1)))
        return new_path

    def draw(self, path, new_path, ws):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d
        ax = plt.figure().add_subplot(projection='3d')

        ax.set_xlim(ws[0], ws[1])
        ax.set_ylim(ws[0], ws[1])
        ax.set_zlim(-5, 5)
        ax.grid()

        # Creating a sphere of radius=safety_radius
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = self.safety_radius*np.outer(np.cos(u), np.sin(v))
        y = self.safety_radius*np.outer(np.sin(u), np.sin(v))
        z = self.safety_radius*np.outer(np.ones(np.size(u)), np.cos(v))

        for obs in self.obstacles:
            x1 = x+obs[0]
            y1 = y+obs[1]
            z1 = z+obs[2]
            ax.plot_surface(x1, y1, z1, rstride=1, cstride=1,
                            cmap='viridis', edgecolor='none')
            # c = plt.Sphere(obs, self.safety_radius, color='#f21010BB')
            # ax.add_patch(c)
        if len(self.edges) != 0:
            for A in self.edges:
                ax.scatter(*A, c='red')
                for B in self.edges[A]:
                    ax.scatter(*B, c='red')
                    ax.plot((A[0], B[0]), (A[1], B[1]),
                            (A[2], B[2]), c='cornflowerblue')
        else:
            for A in self.nodes:
                ax.scatter(*A, c='red')
        ax.scatter(*self.start, c='blue')
        ax.scatter(*self.end, c='green')
        ax.plot(*path.T, c='yellow', linewidth=2)
        ax.plot(*new_path.T, c='black', linewidth=2)
        plt.show()


if __name__ == '__main__':
    def create_obs(nb_of_obs):
        dimension = 3
        nb_of_obs = nb_of_obs//3  # splitted in 3 regions
        T = np.linspace(2.5, 5, nb_of_obs)
        if dimension == 2:
            obs = np.array([2+np.cos(T), 2+np.sin(T)]).T
        else:
            obs = np.array([2+np.cos(T), 2+np.sin(T), 0*T]).T
        obs = np.vstack((obs, -2+np.random.rand(nb_of_obs,
                        dimension), np.random.rand(nb_of_obs, dimension)))
        obs[:, -1] = 0
        return obs

    # np.random.seed(2)
    # obs = create_obs(nb_of_obs=3)
    obs = np.array([[0, 0, 0]])
    circles = Circle(obs, 1)
    points = 4*(np.random.rand(100, 3)-0.5)*2
    points[:, -1] = 0

    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 7))

    positions = np.array(
        [[0, 0, 0], [0.1, 0.1, 0], [-0.3, 0.1, 0], [-0.5, 0.5, 0]])
    circles = Circle(obs, 1)
    of_list = np.flip(circles.points_in_circles(positions))
    A_idx = ([i for i, e in enumerate(np.flip(of_list)) if not e])
    if len(A_idx) != 0:
        A_idx = A_idx[0]
    else:
        from scipy.spatial.transform import Rotation
        t0 = perf_counter()
        nb_of_rays = 10
        rot_vecs = [np.zeros(nb_of_rays), np.zeros(nb_of_rays), np.linspace(
            0, 2*np.pi, nb_of_rays, endpoint=False)]
        rot_vecs = np.array(rot_vecs).T
        print(rot_vecs.shape)
        r = Rotation.from_rotvec(rot_vecs)
        X = positions[-1]
        rays = 0.5*r.apply(np.array([1, 0, 0]))+X
        result = circles.points_in_circles(rays)
        print('total time', 1e3*(perf_counter()-t0), ' ms')
        plt.scatter(rays[result, 0], rays[result, 1], c='red')
        plt.scatter(rays[np.logical_not(result), 0],
                    rays[np.logical_not(result), 1], c='lightgreen')
    A = positions[A_idx]

    p = np.array([-0.5, 0.5, 0])
    for o in obs:
        c = plt.Circle(o[:2], 1, color='red', alpha=0.25)
        ax.add_patch(c)
    ax.plot(*positions[:, :2].T, c='cornflowerblue')
    ax.scatter(*p[:2], color='lightgreen')
    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    ax.set_aspect('equal')
    ax.grid()
    plt.show()


# if __name__ == '__main__':
#     def create_obs(nb_of_obs):
#         dimension = 3
#         nb_of_obs = nb_of_obs//3  # splitted in 3 regions
#         T = np.linspace(2.5, 5, nb_of_obs)
#         if dimension == 2:
#             obs = np.array([2+np.cos(T), 2+np.sin(T)]).T
#         else:
#             obs = np.array([2+np.cos(T), 2+np.sin(T), 0*T]).T
#         obs = np.vstack((obs, -2+np.random.rand(nb_of_obs,
#                         dimension), np.random.rand(nb_of_obs, dimension)))
#         obs[:, -1] = 0
#         return obs

#     # np.random.seed(2)
#     obs = create_obs(nb_of_obs=7)
#     t0 = perf_counter()

#     circles = Circle(obs, 1)
#     points = 4*(np.random.rand(100, 3)-0.5)*2
#     points[:, -1] = 0
#     result = circles.points_in_circles(points)

#     points_good = points[np.logical_not(result)]
#     points_bad = points[result]
#     import matplotlib.pyplot as plt
#     fig, ax = plt.subplots(figsize=(9, 7))

#     p = np.array([-3, -2, 0])
#     for i in range(len(points)):
#         s = Segment(p, points[i])
#         rs = s.is_intersecting_circles(circles)
#         if rs:
#             ax.plot((p[0], points[i, 0]), (p[1], points[i, 1]),
#                     c='tomato', linewidth=1)
#         else:
#             ax.plot((p[0], points[i, 0]), (p[1], points[i, 1]),
#                     c='lightgreen', linewidth=1)

#     for o in obs:
#         c = plt.Circle(o[:2], 1, color='red', alpha=0.25)
#         ax.add_patch(c)
#     ax.scatter(*points_bad[:, :2].T, c='tomato')
#     ax.scatter(*points_good[:, :2].T, c='lightgreen')
#     ax.set_xlim(-4, 4)
#     ax.set_ylim(-4, 4)
#     ax.set_aspect('equal')
#     ax.grid()
#     plt.show()

#     # t=Tree(start=(0,0,0.5),end=(5,5,0),obstacles=obs,safety_radius=1)
#     # t.rrt()
#     # path=t.dijkstra()
#     # new_path=t.smoothen_path(path,iterations=10)
#     print('total time', 1e3*(perf_counter()-t0), ' ms')
#     # path=new_path=np.zeros((1,3))
#     # t.draw(path,new_path,[-5,6.5])
