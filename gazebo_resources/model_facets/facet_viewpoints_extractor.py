"""
Visual computational geometry and viewpoint path constrained optimization.
Andrew Washburn, Prateek Arora, Nikhil Khedekar
University of Nevada Reno
CS791 Special Topics (Robotics)
Instructor: Christos Papachristos
Fall 2020
"""

# Imports
from os.path import exists
import os
import numpy as np
from scipy.spatial.transform.rotation import Rotation as R
from scipy.cluster import hierarchy as shc
from scipy.spatial.distance import pdist, squareform
from numpy.linalg import norm
import math
from stl import mesh as stl_mesh
from mpl_toolkits import mplot3d
from matplotlib.colors import Normalize
from matplotlib import pyplot, cm
from matplotlib import pyplot as plt
import collada as cl
import open3d as o3d
import csv

# from InvokeLKH import writeTSPLIBfile_FE, run_LKHsolver_cmd, rm_solution_file_cmd, copy_toTSPLIBdir_cmd


# Initialize constants
camera_position = np.array([30, 20, 30])
BIG_BEN = "Mesh/BigBen.stl"
TANK = "Mesh/ElevatedTank.stl"
BOAT = "boat.dae"
INCIDENCE_ANGLE = np.pi / 6  # facet field of view
FOV = np.pi / 3  # camera field of view
DMIN = 5
DMAX = 10
D_CLUSTER = 0.8  # [m]
CWD = os.path.dirname(os.path.abspath(__file__))
# model = TANK  # choose the mesh model
model = BOAT# choose the mesh model



def rand_cmap(nlabels, type='bright', first_color_black=True, last_color_black=False, verbose=True):
    """
    Creates a random colormap to be used together with matplotlib. Useful for segmentation tasks
    :param nlabels: Number of labels (size of colormap)
    :param type: 'bright' for strong colors, 'soft' for pastel colors
    :param first_color_black: Option to use first color as black, True or False
    :param last_color_black: Option to use last color as black, True or False
    :param verbose: Prints the number of labels and shows the colormap. True or False
    :return: colormap for matplotlib
    """
    from matplotlib.colors import LinearSegmentedColormap
    import colorsys
    import numpy as np


    if type not in ('bright', 'soft'):
        print ('Please choose "bright" or "soft" for type')
        return

    if verbose:
        print('Number of labels: ' + str(nlabels))

    # Generate color map for bright colors, based on hsv
    if type == 'bright':
        randHSVcolors = [(np.random.uniform(low=0.0, high=1),
                          np.random.uniform(low=0.2, high=1),
                          np.random.uniform(low=0.9, high=1)) for i in range(nlabels)]

        # Convert HSV list to RGB
        randRGBcolors = []
        for HSVcolor in randHSVcolors:
            randRGBcolors.append(colorsys.hsv_to_rgb(HSVcolor[0], HSVcolor[1], HSVcolor[2]))

        if first_color_black:
            randRGBcolors[0] = [0, 0, 0]

        if last_color_black:
            randRGBcolors[-1] = [0, 0, 0]

        random_colormap = LinearSegmentedColormap.from_list('new_map', randRGBcolors, N=nlabels)

    # Generate soft pastel colors, by limiting the RGB spectrum
    if type == 'soft':
        low = 0.6
        high = 0.95
        randRGBcolors = [(np.random.uniform(low=low, high=high),
                          np.random.uniform(low=low, high=high),
                          np.random.uniform(low=low, high=high)) for i in range(nlabels)]

        if first_color_black:
            randRGBcolors[0] = [0, 0, 0]

        if last_color_black:
            randRGBcolors[-1] = [0, 0, 0]
        random_colormap = LinearSegmentedColormap.from_list('new_map', randRGBcolors, N=nlabels)

    # Display colorbar
    if verbose:
        from matplotlib import colors, colorbar
        from matplotlib import pyplot as plt
        fig, ax = plt.subplots(1, 1, figsize=(15, 0.5))

        bounds = np.linspace(0, nlabels, nlabels + 1)
        norm = colors.BoundaryNorm(bounds, nlabels)

        cb = colorbar.ColorbarBase(ax, cmap=random_colormap, norm=norm, spacing='proportional', ticks=None,
                                   boundaries=bounds, format='%1i', orientation=u'horizontal')

    return random_colormap


# Helpers
def normalize(vector):
    return vector / np.linalg.norm(vector, axis=1, keepdims=True)


def dot_v(directions1, directions2):
    return np.sum(directions1 * directions2, axis=1)


def incidence_plane(facet, normal):
    """
    facets is a (3 x 3) array of triangle points in 3D space
    normals is (3, ) array corresponds to each triangle's outward normals
    """
    plane_normals = np.zeros((3, 3))
    for p0, p1 in ((0, 1), (1, 2), (2, 0)):
        q = facet[p1] - facet[p0]
        q = q / norm(q)
        m = R.from_rotvec(INCIDENCE_ANGLE * q)  # euler vector
        n = m.apply(normal)
        plane_normals[p0] = n
    return plane_normals


def visible_facets(viewpoint, mesh):
    plane_origin = np.array([0, 0, viewpoint[2]])
    # compute heading towards the xy origin
    camera_xy_direction = (plane_origin - viewpoint) / norm(plane_origin - viewpoint)

    # Calculate direction from the Camera to each facet's center
    unit_vectors = normalize(mesh.v0 - viewpoint)
    camera_angles = np.arccos(dot_v(camera_xy_direction, unit_vectors))
    visible_facets_idx = np.argwhere(camera_angles <= FOV)

    normals = dot_v(mesh.normals, unit_vectors) # values < 0 are pointing towards camera
    theta = np.arccos(normals)
    feasible_facets_idx = np.argwhere(theta[visible_facets_idx] >= (np.pi - INCIDENCE_ANGLE))
    return feasible_facets_idx[:,0]

def plot_tsp_path(axes, viewpoints):
    path = []
    with open("TSPLIB/CoveragePlanner.txt") as fh:
        on_tour = False
        for line in fh:
            line = line.strip()
            if on_tour:
                point = int(line)
                if point == -1:
                    on_tour = False
                else:
                    path.append(point - 1)

            elif not on_tour and line == "TOUR_SECTION":
                on_tour = True
            
            # print(line, "\t| ", on_tour)
    # plot TSP path lines
    for i in range(n-1):
        p = path[i]
        pn = path[i+1]
        axes.plot(
            xs=[viewpoints[p, 0], viewpoints[pn, 0]],
            ys=[viewpoints[p, 1], viewpoints[pn, 1]],
            zs=[viewpoints[p, 2], viewpoints[pn, 2]],
            color='green'
        )
def get_face_normals(v1, v2, v3, vn):
    """
    v1 = (N,3) array with each row being 3D corrdinates of the 1st vertex of a triangle
    v2 = (N,3) array with each row being 3D corrdinates of the 2nd vertex of a triangle
    vn = (N,3) array with each row being 3D corrdinates of the 1st vertex normal of a triangle
    """
    n = np.cross(v3-v1, v2-v1)
    # print("first 10 normals = {}".format(n[:10]))\
    # perform dot product between normal and vertex normal to determine correct
    # direction of normal
    dot_prod = np.sum(np.multiply(n, vn),axis=1)
    # get array representing direction of normal
    sign_arr = dot_prod > 0
    sign_arr = sign_arr.astype(np.float32)
    # 0 should represent negative direction, so setting to -1
    sign_arr[sign_arr==0.] = -1.
    # tiling sign for easy multiplication using numpy
    sign_tile = np.tile(sign_arr,(3,1))
    # print("sign_tile shape = {}".format(sign_tile.shape))
    sign_tile = sign_tile.transpose()
    # print("sign_tile = {}".format(sign_tile))
    n_out = np.multiply(n, sign_tile)
    norm = np.linalg.norm(n_out,axis=1)
    norm_tile = np.tile(norm,(3,1)).transpose()
    n_out_hat = n_out/norm_tile
    return n_out_hat

def load_dae_mesh(model):
    if not exists(model):
        raise ImportError("{} not found.".format(model))
    mesh = cl.Collada(model)

    # get the first geometry in the first mesh
    geom = mesh.geometries[0]
    # get primitives
    triset_1= geom.primitives[0]
    triset_2 = geom.primitives[1]
    # get vertices of shape (N,3,3)
    vertices = triset_1.vertex[triset_1.vertex_index]
    vertices = np.concatenate((vertices, triset_2.vertex[triset_2.vertex_index]), axis=0)
    print("vertices shape = {}".format(vertices.shape))
    vertices_flat = vertices.reshape(-1,9)
    print("vertex_flat shape = {}".format(vertices_flat.shape))
    print("first 3 of vertex_flat = {}".format(vertices_flat[0:3,:]))
    v1 = vertices_flat[:,0:3]
    v2 = vertices_flat[:,3:6]
    v3 = vertices_flat[:,6:9]
    print("first 3 of v1 = {}\nv2 = {}\nv3 = {}".format(v1[0:3,:],v2[0:3,:],v3[0:3,:]))
    centers = np.mean(vertices,axis=1)
    # normals = 
    normals = np.concatenate((triset_1.normal[triset_1.normal_index], triset_2.normal[triset_2.normal_index] ), axis=0)
    print("normal shape = {}".format(normals.shape))
    normals_flat = normals.reshape(-1,9)
    vn = normals_flat[:,0:3]
    face_normals = get_face_normals(v1,v2,v3,vn)
    print("face_normals shape = {}".format(face_normals.shape))
    # normals = np.mean(normals, axis=1)
    # normals = normals/ np.linalg.norm(normals, axis=1)
    # viewpoints = normals + vertices
    # viewpoints = viewpoints.reshape((-1, 3))
    # print("viewpoints shape = {}".format(viewpoints.shape))
    # viewpoints = centers
    return [triset_1, triset_2], vertices, centers, face_normals 

def plot_3d_mesh(triset, centers, normals):
    # Create a new plot
    figure = pyplot.figure()
    axes = mplot3d.Axes3D(figure)

    # Add polygon with view color to matplotlib figure
    if isinstance(triset,list):
        for each in triset:
            polygon = mplot3d.art3d.Poly3DCollection(each.vertex[each.vertex_index])
            axes.add_collection3d(polygon)
        scale = triset[0].vertex[triset[0].vertex_index].flatten()
    else:
        polygon = mplot3d.art3d.Poly3DCollection(triset.vertex[triset.vertex_index])
        axes.add_collection3d(polygon)
        scale = triset.vertex[triset.vertex_index].flatten()
    # normals_list = []
    # for i in range(len(centers)):
        # end = centers[i,:]+normals[i,:]
        # line3d = zip(centers[i,:].tolist(), end.tolist())
        # print("line3d = {}".format(line3d))
        # normals_list.append(line3d)
    # scale = [10.0,10.0,10.0]
    line3d = (zip(centers, centers+normals))
    norm_class = Normalize(0,scale[0])
    line3d_plot = mplot3d.art3d.Line3DCollection(line3d, colors='k', linewidths=0.2,norm=norm_class)
    axes.add_collection3d(line3d_plot)
    # axes.scatter(xs=viewpoints[:, 0], ys=viewpoints[:, 1], zs=viewpoints[:, 2], marker='o', c='r')

    # Auto scale to the mesh size
    axes.auto_scale_xyz(scale, scale, scale)
    axes.set_xlabel('X')
    axes.set_ylabel('Y')
    axes.set_zlabel('Z')
    # axes.azim = -160
    # axes.dist = 7
    # axes.elev = -3
    # print('ax.azim = {}'.format(axes.azim))
    # print('ax.dist = {}'.format(axes.dist))
    # print('ax.elev = {}'.format(axes.elev))

    # Show the plot to the screen
    pyplot.show()

def load_mesh(model):
    """
    Load the STL files and add the vectors to the plot
    
    Returns: 

    mesh: Mesh object from numpy-stl   
    facets: (n, 3, 3) coordinate array of triangular facets
    unit_normals: unit normals for each facet
    mesh_centers: numpy array of facet center coordinates
    n: number of facets
    """
    if not exists(os.path.join(CWD, model)):
        raise ImportError("{} not found in current file folder.".format(model))

    your_mesh = stl_mesh.Mesh.from_file(os.path.join(CWD, model))
    n = len(your_mesh.points)

    # Compute triangle mesh center
    mesh_centers = np.stack([np.average(your_mesh.x, axis=1), 
                            np.average(your_mesh.y, axis=1), 
                            np.average(your_mesh.z, axis=1)], 
                            axis=-1)

    # Compute incidence planes
    facets = your_mesh.points.reshape(-1, 3, 3)
    unit_normals = your_mesh.normals / np.linalg.norm(your_mesh.normals, axis=1)[:, None]

    return your_mesh, facets, unit_normals, mesh_centers, n


def create_obstacles(facets):
    """
    Create Obstaces in 2D
    Returns 2D array of XY obstacle points
    """
    obstacles = []
    for f in facets:
        for p0, p1 in ((0, 1), (1, 2), (2, 0)):
            dx = f[p1, 0] - f[p0, 0]
            dy = f[p1, 1] - f[p0, 1]
            d = np.hypot(dx, dy)
            theta = np.arctan2(dy, dx)
            x = f[p0, 0:2]
            
            D = 0.25
            nstep = int(d / D)
            obs = np.ones((nstep, 2)) * x
            d_vec = np.vstack([np.arange(nstep), np.arange(nstep)]).T * np.array([D*np.cos(theta), D*np.sin(theta)])
            obs = obs + d_vec
            obstacles.append(obs)

    obstacles = np.concatenate(obstacles)
    return obstacles


def obstacle_perimeter(obstacles):
    """
    Create obstacle perimiter 
    """
    perim = 5
    D = 0.25
    xmin = np.min(obstacles[:,0]) - perim  # [m]
    xmax = np.max(obstacles[:,0]) + perim  # [m]
    ymin = np.min(obstacles[:,1]) - perim  # [m]
    ymax = np.max(obstacles[:,1]) + perim  # [m]

    top = np.arange(xmin, xmax, step=D).reshape(-1,1)
    top = np.concatenate([top, np.ones(top.shape) * ymax], axis=1)
    right = np.arange(ymax, ymin, step=-D).reshape(-1,1)
    right = np.concatenate([np.ones(right.shape) * xmax, right], axis=1)
    bottom = np.arange(xmax, xmin, step=-D).reshape(-1,1)
    bottom = np.concatenate([bottom, np.ones(bottom.shape) * ymin], axis=1)
    left = np.arange(ymin, ymax, step=D).reshape(-1,1)
    left = np.concatenate([np.ones(left.shape) * xmin, left], axis=1)

    obstacles = np.concatenate([obstacles, np.concatenate([top, right, bottom, left])])
    return obstacles


def viewpoint_clusters(viewpoints, d_cluster=D_CLUSTER):
    """
    Takes points and uses complete hierarchial clustering to return cluster centers in 2D
    \nReturns:
    cluster_groups: a list of group numbers for each viewpoint
    cluster_centers: a 2D XY array of cluster centers
    """
    # TODO: use PRM to compute path distances.
    distMatrix = pdist(viewpoints)
    Z = shc.complete(distMatrix)
    cluster_groups = shc.fcluster(Z, D_CLUSTER, criterion='distance')

    n_clusters = max(cluster_groups)
    cluster_centers = np.zeros((n_clusters, 2))
    for c in range(n_clusters):
        group = cluster_groups == c+1
        view_group = viewpoints[group, 0:2]
        cluster_centers[c] = np.median(view_group, axis=0)

    return cluster_groups, cluster_centers


def create_viewpoints(mesh_model, incidence_angle=INCIDENCE_ANGLE, dmin=.1, dmax=2):
    """
    Given a mesh model, create a viewpoint 1m away from the first vertex.
    Constraints:
    Below height threshold.
    Above the ground.
    Not inside an object.
    Within incidence angle of a facet
    """
    unit_norm = mesh_model.normals / np.linalg.norm(mesh_model.normals, axis=1)[:, None]

    # For all points in the mesh calculate a rectangular region to sample points from
    viewpoints = mesh_model.v0 + unit_norm
    normal = viewpoints - mesh_model.v0
    return viewpoints, normal


def plot_3d_object_viewpoints(mesh_model, viewpoints):
    # Create a new plot
    figure = pyplot.figure()
    axes = mplot3d.Axes3D(figure)

    # Add polygon with view color to matplotlib figure
    polygon = mplot3d.art3d.Poly3DCollection(mesh_model.vectors)
    axes.add_collection3d(polygon)
    axes.scatter(xs=viewpoints[:, 0], ys=viewpoints[:, 1], zs=viewpoints[:, 2], marker='o', c='r')

    # Auto scale to the mesh size
    scale = mesh_model.points.flatten()
    axes.auto_scale_xyz(scale, scale, scale)
    axes.set_xlabel('X')
    axes.set_ylabel('Y')
    axes.set_zlabel('Z')
    # Show the plot to the screen
    pyplot.show()

def get_quat_from_norm(normals):
    init = np.array([1., 0., 0.]).astype(np.float32)
    q_arr = []
    for i in range(len(normals)):
        a = np.cross(init,normals[i,:])
        q = np.array([0,0,0,1]).astype(np.float32)
        q[0] = a[0]
        q[1] = a[1]
        q[2] = a[2]
        q[3] = np.dot(init,normals[i,:])
        q = q/np.linalg.norm(q)
        q_arr.append(q)
    q_arr = np.array(q_arr)
    return q_arr

def main():

     # Load the model and get triangles
    triset, vertices, centers, normals = load_dae_mesh(model)
    print()
    quat = get_quat_from_norm(normals)
    print("quat shape: ", quat.shape)
    print("center shaep: ", centers.shape)
    with open('boat.csv', 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow(['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        for i in range(len(centers)):
            writer.writerow([centers[i,0], centers[i,1], centers[i,2], quat[i,0], quat[i,1], quat[i,2], quat[i,3]])
    # plot the mesh
    # plot_3d_mesh(triset, centers, normals)


if __name__ == "__main__":
    main()