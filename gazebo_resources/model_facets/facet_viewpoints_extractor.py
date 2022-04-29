"""
Viewpoint extraction from dae file
Prateek Arora
University of Nevada Reno
CS791 Special Topics (Robotics)
Instructor: Christos Papachristos
Spring 2022
"""

# Imports
from os.path import exists
import os
import numpy as np
from mpl_toolkits import mplot3d
from matplotlib.colors import Normalize
from matplotlib import pyplot
import collada as cl
import open3d as o3d # for visualization
import csv
import copy
import argparse


def get_face_normals(v1, v2, v3, vn):
    """
    v1 = (N,3) array with each row being 3D corrdinates of the 1st vertex of a triangle
    v2 = (N,3) array with each row being 3D corrdinates of the 2nd vertex of a triangle
    vn = (N,3) array with each row being 3D corrdinates of the 1st vertex normal of a triangle
    """
    n = np.cross(v3-v1, v2-v1)
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
    """
    Input: 
        model: path to .dae file
    Output:
        primitives_list: set of primitives in the model geometry 
        vertices: (N,3,3) array with each (3,3) matrix being 3D corrdinates of 
                   the 3 vertices of a triangle
        centers_txn: (N,3) array with each row being 3D corrdinates of the 
                    center of a triangle
        face_normals_txn: (N,3) array with each row being 3D corrdinates of the
                           face normal of a triangle
    """
    if not exists(model):
        raise ImportError("{} not found.".format(model))
    mesh = cl.Collada(model)
    mesh_copy = copy.deepcopy(mesh)
    # print("txn = {}".format(mesh.scene.nodes[0].tranforms[0].matrix))
    mesh_copy.scene.nodes[0].transforms[0].matrix = np.eye(4)
    # mesh_b.scene.nodes[0].transforms[0].matrix = np.eye(4)

    # get the first geometry in the first mesh
    geom = mesh_copy.geometries[0]
    # get all primitives in a list so that it can be iterated
    primitives_list = []
    for primitives in mesh.geometries[0].primitives:
        primitives_list.append(primitives)

    # get vertices from all primitives into a single vertex array
    # of shape (N,3,3)
    for i, primitives in enumerate(primitives_list):
        if i == 0:
            vertices = primitives.vertex[primitives.vertex_index]
        else:
            vertices = np.concatenate((vertices, primitives.vertex[primitives.vertex_index]), axis=0)
    # flatten vertices of each face for ease of access
    vertices_flat = vertices.reshape(-1,9)
    # first vertex of each face
    v1 = vertices_flat[:,0:3]
    # second vertex of each face
    v2 = vertices_flat[:,3:6]
    # third vertex of each face
    v3 = vertices_flat[:,6:9]

    # calculate centers by averaging vertices (N,3)
    centers = np.mean(vertices,axis=1)
    
    # get vertex normals from all primitives into a single array
    # of shape (N,3,3)
    for i, primitives in enumerate(primitives_list):
        if i == 0:
            normals = primitives.normal[primitives.normal_index]
        else:
            normals= np.concatenate((normals, primitives.normal[primitives.normal_index]), axis=0)
    # flatten normals of each face for ease of access
    normals_flat = normals.reshape(-1,9)
    # get first vertex normal of each face
    vn = normals_flat[:,0:3]

    # compute face normals
    face_normals = get_face_normals(v1,v2,v3,vn)
    # get transformation matrix
    T = mesh.scene.nodes[0].transforms[0].matrix
    # extract roation matrix from transformation matrix
    R = T[0:3,0:3]
    # convert centers into homogeneous coordinates by appending 1 at the end of
    # each vector (row)
    centers_homogeneous = np.hstack((centers,np.ones((len(centers),1))))
    # reshape to center_homogeneous to (3,N) for matrix multiplication
    # and then perform matrix multiplication (transformation)
    centers_txn = T@(centers_homogeneous.transpose())
    centers_txn = centers_txn.transpose()
    # unhomogenize centers_txn by removing last column consisting of 1s
    centers_txn = centers_txn[:,:-1]
    
    # rotate face normals
    face_normals_txn = (R@(face_normals.transpose())).transpose()

    return primitives_list, vertices, centers_txn, face_normals_txn

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

    # Show the plot to the screen
    pyplot.show()


def get_quat_from_norm(normals):
    reference = np.array([1., 0., 0.]).astype(np.float32)
    q_arr = []
    for i in range(len(normals)):
        n = np.cross(reference, normals[i,:])
        n_norm = np.linalg.norm(n)
        n = n/n_norm

        theta = np.arctan2(n_norm, np.inner(reference, normals[i,:]))
        
        sin_theta_over_2 = np.sin(theta/2)
        cos_theta_over_2 = np.cos(theta/2)
        vec = n*sin_theta_over_2
        q = np.array([0,0,0,1]).astype(np.float32)
        q[0] = n[0]
        q[1] = n[1]
        q[2] = n[2]
        q[3] = cos_theta_over_2
        q = q/np.linalg.norm(q)
        q_arr.append(q)
    q_arr = np.array(q_arr)
    return q_arr

def plot_centers_and_normal_line(centers, normals):
    points = np.concatenate([centers, centers + normals], axis=0)
    # print(points.shape)
    # print(centers[len(centers)-1])
    lines = []
    for i in range(len(normals)):
        lines.append([i, len(normals)+i])
    lines = np.array(lines)
    colors = [[1, 0, 0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([line_set])
    # print("lines = {}".format(lines[:2]))

def plot_centers_and_normals(centers, quat):
    all_meshes = []
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.50, origin=[0, 0, 0])
    all_meshes.append(frame)
    centers = centers*100
    for i in range(len(centers)):
        mesh = o3d.geometry.TriangleMesh.create_arrow()#cylinder_radius=0.2, cone_radius=0.3, cylinder_height=5.0, cone_height=0.2, resolution=20, cylinder_split=4, cone_split=1)
        mesh_r = copy.deepcopy(mesh)
        mesh_r = mesh_r.translate([centers[i,0],centers[i,1],centers[i,2]])
        quat_r = np.array([quat[i,0],quat[i,1],quat[i,2],quat[i,3]])
        # quat_r = np.array([quat[i,3],quat[i,0],quat[i,1],quat[i,2]])
        mesh_r.rotate(mesh.get_rotation_matrix_from_quaternion(quat_r))
        all_meshes.append(mesh_r)

    o3d.visualization.draw_geometries(all_meshes)

def main():
    parser = argparse.ArgumentParser(description='Generate normals and centers for a mesh file')
    parser.add_argument('-m', '--mesh', help='Input path to mesh file', default='boat.dae')
    args = parser.parse_args()
    model = args.mesh
     # Load the model and get triangles
    print("Loading mesh...")
    triset, vertices, centers, normals = load_dae_mesh(model)
    print("Done loading mesh.")
    quat = get_quat_from_norm(normals)
    print("Writing to csv file...")
    with open('boat.csv', 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow(['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        for i in range(len(centers)):
            writer.writerow([centers[i,0], centers[i,1], centers[i,2], quat[i,0], quat[i,1], quat[i,2], quat[i,3]])
    print("Done.")
    # plot_centers_and_normals(centers, quat)
    # plot_centers_and_normal_line(centers, normals)
    # plot the mesh
    # plot_3d_mesh(triset, centers, normals)


if __name__ == "__main__":
    main()