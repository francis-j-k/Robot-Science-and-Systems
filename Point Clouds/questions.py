from typing import Tuple
import numpy as np

import utils


def q1_a(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a least squares plane by taking the Eigen values and vectors
    of the sample covariance matrix

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''
    #center is calculated
    center = np.mean(P,axis=0)
    #covariance is calculated
    covariance = np.cov(P.T)
    U, S, V = np.linalg.svd(covariance)
    normal=V[2]
    #Normal is computed
    return normal, center
    #pass


def q1_c(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a plane using RANSAC

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''
 
    num_points = P.shape[0]
    inlier_indices = []
    plane_normal = np.zeros(3)
    plane_center = np.zeros(3)

    for iteration in range(1000):
        #select random points
        random_indices = np.random.choice(range(num_points), 3)
        #calculating the Normal
        curr_normal = np.cross(P[random_indices[1]] - P[random_indices[0]], 
                               P[random_indices[2]] - P[random_indices[0]])
        norm = np.linalg.norm(curr_normal)
        if norm < np.finfo(float).eps:
            continue
        curr_normal /= norm
        #Computing the distance
        distance = np.abs(np.dot(P - P[random_indices[0]], curr_normal))
        #finding the inliers
        inliers = np.where(distance < 0.01)[0]
        #getting the best inliers
        if len(inliers) > len(inlier_indices):
            inlier_indices = inliers
            plane_normal = curr_normal
            plane_center = P[random_indices[0]]
    return plane_normal, plane_center
    #pass


def q2(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, float]:
    '''
    Localize a sphere in the point cloud. Given a point cloud as input, this function should locate the position and radius
    of a sphere

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space
    N : np.ndarray
        Nx3 matrix denoting normals of pointcloud

    Returns
    -------
    center : np.ndarray
        array of shape (3,) denoting sphere center
    radius : float
        scalar radius of sphere
    '''
    
    num_points = P.shape[0]
    best_inliers = []
    best_center = np.zeros(3)
    best_radius = 0
    
    for i in range(1000):
        #Pick a point randomly
        random_idx = np.random.choice(range(num_points), 1)
        #get surface normal
        curr_normal = N[random_idx]
        #sample radius between 5cm and 11cm
        curr_radius = np.random.uniform(0.05, 0.11)
        #compute center of sphere using sample points and radius
        curr_center = P[random_idx] + curr_radius * curr_normal
        #calculate distance from center to all points
        distances = np.linalg.norm(P - curr_center, axis=1)
        #finding the inliers
        inlier_idx = np.where(np.logical_and(distances > curr_radius - 0.002, distances < curr_radius + 0.002))[0]
        #update the inliers
        if len(inlier_idx) > len(best_inliers):
            best_inliers = inlier_idx
            best_center = curr_center
            best_radius = curr_radius
    
    best_center = best_center.reshape((3, 1))
    return best_center, best_radius
    #pass


def q3(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    # get number of points
    num_points = P.shape[0]

    # Initialize variables 
    B_inlier_indices = []
    B_center = np.zeros(3)
    B_axis = np.zeros(3)
    B_radius = 0

    # loop for sphere-fitting procedure
    for i in range(1000):
        # Choosing a random radius and indices
        current_radius = np.random.uniform(0.05, 0.1)
        index_pair = np.random.choice(range(num_points), 2, replace=False)

        # Computing the normal vectors corresponding to the chosen indices
        current_normal1 = N[index_pair[0]]
        current_normal2 = N[index_pair[1]]

        # Normalize 
        current_normal1 /= np.linalg.norm(current_normal1)
        current_normal2 /= np.linalg.norm(current_normal2)

        # Computing the axis and center of the sphere
        current_axis = np.cross(current_normal1, current_normal2)
        current_center = P[index_pair[0]] + current_radius * current_normal1

        # Computing the projection of the P onto the plane perpendicular to the axis
        n = np.eye(3) - np.outer(current_axis, current_axis)
        projected_points = np.matmul(n, P.T).T
        projected_center = np.matmul(n, current_center.T).T
        d = np.linalg.norm(projected_points - projected_center, axis=1)

        # Finding the inliers
        inlier = np.where(np.logical_and(np.abs(d) < current_radius + 0.00005,
                                             np.abs(d) > current_radius - 0.00005))[0]

        # update the sphere parameters with best inliers
        if len(inlier) > len(B_inlier_indices):
            B_inlier_indices = inlier
            B_center = current_center
            B_axis = current_axis
            B_radius = current_radius
    B_center = B_center.reshape((3, 1))
    return B_center, B_axis, B_radius
    

    
def q4_a(M: np.ndarray, D: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Find transformation T such that D = T @ M. This assumes that M and D are
    corresponding (i.e. M[i] and D[i] correspond to same point)

    Attributes
    ----------
    M : np.ndarray
        Nx3 matrix of points
    D : np.ndarray
        Nx3 matrix of points

    Returns
    -------
    T : np.ndarray
        4x4 homogenous transformation matrix

    Hint
    ----
    use `np.linalg.svd` to perform singular value decomposition
    '''
     # center the two point clouds
    M_center = M - np.mean(M, axis=0)
    D_center = D - np.mean(D, axis=0)
    # perform SVD to get min t and rotation
    mat = np.matmul(np.transpose(D_center), M_center)
    u, s, vh = np.linalg.svd(mat)
    # rotation = UV^T
    rot = np.matmul(u,vh)
    # optimal translation vector t 
    t = np.matmul(-rot,np.transpose(np.mean(M, axis=0))) + np.transpose(np.mean(D, axis=0))
    # construct the transformation matrix T
    T = np.eye(4)
    T[0:3,0:3] = rot
    T[0:3,3] = t

    return T
    #pass


def q4_c(M: np.ndarray, D: np.ndarray) -> np.ndarray:
    
    i = 0
    # centroid
    p = np.mean(M, axis=0)
    x = np.mean(D, axis=0)   
    # translation
    translation = x.T - p.T
    rotation = np.eye(3)  
    # DEclaring the Transformation matrix
    final_Transformation = np.eye(4)
    final_Transformation[:3, :3] = rotation
    final_Transformation[:3, 3] = translation
    M = M + translation
    previous_error = 0
    error = np.inf
    while error>0.00001 and i < 50:
        # closest points in M for each point in D is found
        closest_point_in_M = np.zeros((D.shape[0], 3))
        for j in range(D.shape[0]):
            dist = np.linalg.norm(M - D[j], axis=1)
            closest_point_in_M[j] = M[np.argmin(dist)]
        M = closest_point_in_M
        T = q4_a(M, D)              
        # transformation matrix being update 
        final_Transformation = np.matmul(T, final_Transformation)
        # transformation to M
        M = utils.apply_transform(M, T)
        # Updating error in the loop
        curr_error = np.linalg.norm(M - D)       
        error = np.abs(curr_error - previous_error)
        print(f'Iteration {i}: error = {error:.6f}')
        previous_error = curr_error
        i += 1        
    return final_Transformation