#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <cmath>
#include "gen_utilities/utilities.hpp"
#include "gen_utilities/transformation_utilities.hpp"
#include "gen_utilities/NPAM_utilities.hpp"

///////////////////////////////////////////////////////////

Eigen::MatrixXd NPAM::rotate_pts(Eigen::MatrixXd pts, double theta, double x_avg, double y_avg)
{
	// Function for applying rotation to points about their centroid
	// INPUT : xyz points, rotation angle, x-avg and y-avg
	// OUTPUT : rotated points about their centroid

	// translate points so that origin matches with the centroid
	pts.block(0,0,pts.rows(),1) = pts.block(0,0,pts.rows(),1).array() - x_avg;
	pts.block(0,1,pts.rows(),1) = pts.block(0,1,pts.rows(),1).array() - y_avg;

	// rotate points about centroid
	Eigen::Matrix3d r = rtf::rot_z(theta);
	Eigen::Vector3d t;
	t << 0,0,0;
	Eigen::Matrix4d T = rtf::hom_T(t, r);
	Eigen::MatrixXd pts_before_T(pts.rows(),pts.cols()+1);
	pts_before_T << pts,Eigen::MatrixXd::Constant(pts.rows(),1,0);
	Eigen::MatrixXd	pts_T = rtf::apply_transformation(pts_before_T, T); 

	// translate points back to the original position
	pts_T.block(0,0,pts_T.rows(),1) = pts_T.block(0,0,pts_T.rows(),1).array() + x_avg;
	pts_T.block(0,1,pts_T.rows(),1) = pts_T.block(0,1,pts_T.rows(),1).array() + y_avg;
	return pts_T.block(0,0,pts_T.rows(),2);
}

///////////////////////////////////////////////////////////

int NPAM::number_of_layers(const Eigen::MatrixXd& v, const Eigen::MatrixXd& f, const Eigen::MatrixXd& n, double pathgap_z)
{
	Eigen::MatrixXd n_new(n.rows(),4);
    n_new.block(0,0,n_new.rows(),3) = (((n.array()*100.0).cast<int>()).cast<double>().array()/100.0);
    n_new.block(0,3,n_new.rows(),1) = ut::linsp(0,n_new.rows()-1,1); 
    
    // sorting normals for top and bottom face
    Eigen::MatrixXd nn_i = Eigen::MatrixXd::Constant(n_new.rows(),n_new.cols(),0);
    Eigen::MatrixXd np_i = Eigen::MatrixXd::Constant(n_new.rows(),n_new.cols(),0);
    long int nn_idx = 0;
    long int np_idx = 0;
    for (long int i=0;i<n_new.rows();++i)
    {
    	if (n_new(i,2)<0)
    	{
    		nn_i.row(nn_idx) = n_new.row(i);
    		++nn_idx;
    	}
    	else if (n_new(i,2)>0)
    	{
    		np_i.row(np_idx) = n_new.row(i);
    		++np_idx;
    	}
    }
    Eigen::MatrixXd nn = nn_i.block(0,0,nn_idx,nn_i.cols());
	Eigen::MatrixXd np = np_i.block(0,0,np_idx,np_i.cols());

    // getting normals which are opposite ot each other and forming pairs of parallel faces 
	Eigen::MatrixXd p11(1,3);
	Eigen::MatrixXd p12(1,3);
	Eigen::MatrixXd p13(1,3);
	Eigen::MatrixXd p21(1,3);
	Eigen::MatrixXd nn_row(1,3);
	double a = 0;
	double b = 0;
	double c = 0;
	double d = 0;
	std::vector<double> t;
	for (long int i=0;i<nn.rows();++i)
	{
		nn_row = -nn.block(i,0,1,3);
		std::vector<int> idx = ut::ismember(np.block(0,0,np.rows(),3),nn_row);
		std::vector<int> store_idx = ut::find_idx(idx);
		if (store_idx.size()!=0)
		{
			// calculating the gap between parallel faces by plane to plane distance formaula
			long int pair[2];
			pair[0] = np(store_idx[0],3);
			pair[1] = nn(i,3);
			p11 = v.row(f(pair[0],0)-1);
			p12 = v.row(f(pair[0],1)-1);
			p13 = v.row(f(pair[0],2)-1);
			p21 = v.row(f(pair[1],0)-1);
			a = ((p12(0,1)-p11(0,1))*(p13(0,2)-p11(0,2)))-((p13(0,1)-p11(0,1))*(p12(0,2)-p11(0,2)));
        	b = ((p12(0,2)-p11(0,2))*(p13(0,0)-p11(0,0)))-((p13(0,2)-p11(0,2))*(p12(0,0)-p11(0,0)));
        	c = ((p12(0,0)-p11(0,0))*(p13(0,1)-p11(0,1)))-((p13(0,0)-p11(0,0))*(p12(0,1)-p11(0,1)));
        	d = -(a*p11(0,0))-(b*p11(0,1))-(c*p11(0,2));
        	t.push_back(fabs((a*p21(0,0)+b*p21(0,1)+c*p21(0,2)+d)/sqrt((a*a)+(b*b)+(c*c))));
		}
	}
	double t_f = ut::median(t);
	int num_of_layers = round(t_f/pathgap_z);
	if (num_of_layers==0)
	{
		num_of_layers=1;
	}
	return num_of_layers;
}

///////////////////////////////////////////////////////////

Eigen::MatrixXd NPAM::generate_grid_points(double pathgap_x, double pathgap_y, double xmin, double ymin, double xmax, double ymax, double hatch_angle)
{
	// Function to generate the uniform mesh grid of points along the x-y plane
	// INPUT = gap between the adjacent points and maximum value in x and y direction
	// OUTPUT = All points consisting the uniform grid

	Eigen::VectorXd j = ut::linsp(floor(ymin),ceil(ymax),pathgap_y);
	Eigen::VectorXd i_val = ut::linsp(floor(xmin),ceil(xmax),pathgap_x);
	Eigen::MatrixXd	pts = Eigen::MatrixXd::Constant(j.rows()*i_val.rows(),2,0);
	long int st_pt = 0;
	for (long int i=0;i<i_val.rows();++i)
	{
		pts.block(st_pt,0,j.rows(),1) = i_val(i)*Eigen::MatrixXd::Constant(j.rows(),1,1);
		pts.block(st_pt,1,j.rows(),1) = j.block(0,0,j.rows(),j.cols());
		st_pt = st_pt + j.rows();
	}
	return pts;	
}

///////////////////////////////////////////////////////////

Eigen::MatrixXd NPAM::identify_bottom_layer(const Eigen::MatrixXd& v, const Eigen::MatrixXd& f, const Eigen::MatrixXd& n)
{
	// This function identifies the vertices of bottom layer first based on the normal direction
	// INPUT = vertices (3n-by-3 matrix), faces (n-by-3 matrix) and normals (n-by-3 matrix) generated by the STL file.
	// OUTPUT = vertices of the of the bottom surface (m-by-3 matrix)

	Eigen::VectorXd indexv = ut::linsp(1,v.rows(),1);		// indexing vertices
	Eigen::VectorXd indexn = ut::linsp(1,n.rows(),1);		// indexing normals...same as indexing faces
	Eigen::MatrixXd nnew(n.rows(),7);
	nnew << n,f,indexn;
	Eigen::MatrixXd vnew(v.rows(),4);
	vnew << v,indexv; 

	// storing index of only those normals whose z direction is negative,
	// i.e. normal pointing downwards... i.e normals from bottom layer
	Eigen::MatrixXd	store = Eigen::MatrixXd::Constant(nnew.rows(),nnew.cols(),0);
	long int idx_n = 0;
	for (long int i=0;i<nnew.rows();++i)
	{
		if (nnew(i,2)<0)
		{
			store.block(idx_n,0,1,store.cols()) = nnew.block(i,0,1,nnew.cols());
			++idx_n;
		}
	}

	// using index of those normals to get the faces and hence the points associated with those faces
	Eigen::MatrixXd fnew = store.block(0,3,idx_n,3);
	return fnew;
}

///////////////////////////////////////////////////////////

Eigen::MatrixXd NPAM::identify_top_layer(const Eigen::MatrixXd& v, const Eigen::MatrixXd& f, const Eigen::MatrixXd& n)
{
	// This function identifies the vertices of bottom layer first based on the normal direction
	// INPUT = vertices (3n-by-3 matrix), faces (n-by-3 matrix) and normals (n-by-3 matrix) generated by the STL file.
	// OUTPUT = vertices of the of the bottom surface (m-by-3 matrix)

	Eigen::VectorXd indexv = ut::linsp(1,v.rows(),1);		// indexing vertices
	Eigen::VectorXd indexn = ut::linsp(1,n.rows(),1);		// indexing normals...same as indexing faces
	Eigen::MatrixXd nnew(n.rows(),7);
	nnew << n,f,indexn;
	Eigen::MatrixXd vnew(v.rows(),4);
	vnew << v,indexv; 

	// storing index of only those normals whose z direction is positive,
	// i.e. normal pointing upwards... i.e normals from top layer
	Eigen::MatrixXd	store = Eigen::MatrixXd::Constant(nnew.rows(),nnew.cols(),0);
	long int idx_n = 0;
	for (long int i=0;i<nnew.rows();++i)
	{
		if (nnew(i,2)>0)
		{
			store.block(idx_n,0,1,store.cols()) = nnew.block(i,0,1,nnew.cols());
			++idx_n;
		}
	}

	// using index of those normals to get the faces and hence the points associated with those faces
	Eigen::MatrixXd fnew = store.block(0,3,idx_n,3);
	return fnew;
}

///////////////////////////////////////////////////////////

Eigen::MatrixXd	NPAM::project_grid_points(const Eigen::MatrixXd& fnew, const Eigen::MatrixXd& v, const Eigen::MatrixXd& pts, double hatch_angle, double x_avg, double y_avg)
{
	Eigen::MatrixXd p1(1,v.cols());
	Eigen::MatrixXd p2(1,v.cols());
	Eigen::MatrixXd p3(1,v.cols());
	Eigen::MatrixXd	tri(3,v.cols());
	Eigen::MatrixXd	fillpts = Eigen::MatrixXd::Constant(int(1.5*pts.rows()),6,0);
	double a, b, c, d, zval;
	long int fillpts_idx = 0;
	for (long int i=0;i<fnew.rows();++i)
	{
		// vertices for each triangle
		p1 = v.row(fnew(i,0)-1);
		p2 = v.row(fnew(i,1)-1);
		p3 = v.row(fnew(i,2)-1);

		// forming the face with vertices
		tri.block(0,0,1,p1.cols()) = p1;
		tri.block(1,0,1,p1.cols()) = p2;
		tri.block(2,0,1,p1.cols()) = p3;

		// projecting triagles on to the xy plane and 
		// storing all grid points which are inside triangle
		Eigen::MatrixXd in = Eigen::MatrixXd::Constant(pts.rows(),1,0);
		ut::InPoly(pts, tri, in);
		std::vector<int> loc = ut::find_idx(in);
		Eigen::MatrixXd store(loc.size(),pts.cols());
		Eigen::VectorXd storez(loc.size(),1);
		for (int loc_i=0;loc_i<loc.size();++loc_i)
		{
			store.row(loc_i) = pts.row(loc[loc_i]);
		}	
		if (store.rows()!=0)
		{
			// creating plane eqaution to get z value of stored points
			a = ((p2(0,1)-p1(0,1))*(p3(0,2)-p1(0,2)))-((p3(0,1)-p1(0,1))*(p2(0,2)-p1(0,2)));
			b = ((p2(0,2)-p1(0,2))*(p3(0,0)-p1(0,0)))-((p3(0,2)-p1(0,2))*(p2(0,0)-p1(0,0)));
			c = ((p2(0,0)-p1(0,0))*(p3(0,1)-p1(0,1)))-((p3(0,0)-p1(0,0))*(p2(0,1)-p1(0,1)));
			d = -(a*p1(0,0))-(b*p1(0,1))-(c*p1(0,2));
			for (int store_i=0;store_i<store.rows();++store_i)
			{
				zval = (-d-(a*store(store_i,0))-(b*store(store_i,1)))/c;
				if (zval<0)
				{
					storez(store_i,0) = 0;
				}
				else
				{
					storez(store_i,0) = zval;	
				}
			}

			// along with points, storing unit normal 
			// associated with each point from which, tcp orientation is calculated.
			Eigen::MatrixXd n = Eigen::MatrixXd::Constant(store.rows(),3,1);
			n.col(0) = n.col(0).array()*(a/sqrt((a*a)+(b*b)+(c*c)));
			n.col(1) = n.col(1).array()*(b/sqrt((a*a)+(b*b)+(c*c)));
			n.col(2) = n.col(2).array()*(c/sqrt((a*a)+(b*b)+(c*c)));
			fillpts.block(fillpts_idx,0,store.rows(),2) = store;
			fillpts.block(fillpts_idx,2,store.rows(),1) = storez;
			fillpts.block(fillpts_idx,3,store.rows(),3) = -n;
			fillpts_idx = fillpts_idx + store.rows();			
		}
	}	

	// giving negative of hatching angle to make points aligned along reference axes
	Eigen::MatrixXd fillpts_final(fillpts_idx,fillpts.cols());	
	fillpts_final = fillpts.block(0,0,fillpts_idx,fillpts.cols());
	Eigen::MatrixXd fillpts_new = rotate_pts(fillpts_final.block(0,0,fillpts_final.rows(),2),-hatch_angle,x_avg,y_avg);
	fillpts_final.col(0) = fillpts_new.col(0);
	fillpts_final.col(1) = fillpts_new.col(1);
	return fillpts_final; 
}

///////////////////////////////////////////////////////////

Eigen::MatrixXd NPAM::Infill_Path(const Eigen::MatrixXd& fillpts, bool FlipTravel, double space, double hatch_angle, double x_avg, double y_avg, int skip_lines)
{
    // function for creating path from projected points for tcp travel along x direction
    // INPUT: projected points on the surface (equally spaced along x and y axes)
    // OUTPUT: points arranged along 0 degree path with their Rx and Ry value

	// aligning points
	Eigen::MatrixXd storesort = NPAM::align_pts(fillpts);

	// getting rotation angles Rx and Ry...keeping Rz to be 0
    Eigen::VectorXd Rx = -atan(storesort.block(0,4,storesort.rows(),1).array()/storesort.block(0,5,storesort.rows(),1).array())*(180/3.14159);
    Eigen::VectorXd Ry = atan(storesort.block(0,3,storesort.rows(),1).array()/storesort.block(0,5,storesort.rows(),1).array())*(180/3.14159);
    Eigen::MatrixXd storesort0x1(storesort.rows(),5);
    storesort0x1.block(0,0,storesort.rows(),3) = storesort.block(0,0,storesort.rows(),3);
    storesort0x1.block(0,3,storesort.rows(),1) = Rx;
    storesort0x1.block(0,4,storesort.rows(),1) = Ry;

	// skipping pts along line to smoothen motion
    Eigen::MatrixXd storesort0x1tp = NPAM::smoothened_traj_by_pts_skip(storesort0x1, space);

    // skipping the toolpath lines
    Eigen::MatrixXd storesort0x1tp_reduced_lines = NPAM::skip_toolpath_lines(storesort0x1tp,skip_lines);

    // flip the direction of travel
    if (FlipTravel)
    {
        Eigen::MatrixXd storesort0x1tp_rev(storesort0x1tp_reduced_lines.rows(),storesort0x1tp_reduced_lines.cols());
        storesort0x1tp_rev = storesort0x1tp_reduced_lines.colwise().reverse();
        storesort0x1tp = storesort0x1tp_rev;
    }

    //apply hatching angle
    Eigen::MatrixXd storesort0x1tp_new = rotate_pts(storesort0x1tp_reduced_lines.block(0,0,storesort0x1tp_reduced_lines.rows(),2),hatch_angle,x_avg,y_avg);
    storesort0x1tp_reduced_lines.block(0,0,storesort0x1tp_reduced_lines.rows(),2) = storesort0x1tp_new;
    return storesort0x1tp_reduced_lines;
}

///////////////////////////////////////////////////////////

Eigen::MatrixXd NPAM::align_pts(const Eigen::MatrixXd& fillpts)
{
	int dir1 = 1;
    int dir2 = 0;
    Eigen::MatrixXd allpts = fillpts;
    std::vector<std::vector<double> > allpts_vec;
    allpts_vec = ut::mat_to_vec(allpts);
    allpts_vec = ut::SortRows(allpts_vec, dir1);
    allpts = ut::vec_to_mat(allpts_vec);
    int flip  = 0;
    Eigen::MatrixXd storeset = Eigen::MatrixXd::Constant(allpts.rows(),allpts.cols(),0);
    Eigen::MatrixXd storesort = Eigen::MatrixXd::Constant(allpts.rows(),allpts.cols(),0);
    long int storesort_strt = 0;
    long int storeset_idx = 0;
    for (long int i=0;i<allpts.rows()-1;++i)
    {
        if (std::abs(allpts(i,dir1)-allpts(i+1,dir1))<0.00001)
        {
            storeset.row(storeset_idx) = allpts.row(i);
            ++storeset_idx;
        }   
        else
        {
            storeset.row(storeset_idx) = allpts.row(i);
            Eigen::MatrixXd storeset_nz(storeset_idx+1,storeset.cols());
            storeset_nz = storeset.block(0,0,storeset_idx+1,storeset.cols());
            std::vector<std::vector<double> > storeset_nz_vec;
            storeset_nz_vec = ut::mat_to_vec(storeset_nz);
            storeset_nz_vec = ut::SortRows(storeset_nz_vec,dir2);
            storeset_nz = ut::vec_to_mat(storeset_nz_vec);
            if (double(flip)/2==round(double(flip)/2))
            {
                Eigen::MatrixXd storeset_nz_rev(storeset_nz.rows(),storeset_nz.cols()); 
                storeset_nz_rev = storeset_nz.colwise().reverse();
                storeset_nz = storeset_nz_rev;
            }
            flip++;
            storesort.block(storesort_strt,0,storeset_nz.rows(),storeset_nz.cols()) = storeset_nz;
            storesort_strt = storesort_strt + storeset_idx+1;
            storeset_idx = 0;
        }       
    }
    storeset.row(storeset_idx) = allpts.row(allpts.rows()-1);
    Eigen::MatrixXd storeset_nz(storeset_idx+1,storeset.cols());
    storeset_nz = storeset.block(0,0,storeset_idx+1,storeset.cols());
    std::vector<std::vector<double> > storeset_nz_vec;
    storeset_nz_vec = ut::mat_to_vec(storeset_nz);
    storeset_nz_vec = ut::SortRows(storeset_nz_vec,dir2);
    storeset_nz = ut::vec_to_mat(storeset_nz_vec);

    // this is to get the direction of last line travel
    if (double(flip)/2==round(double(flip)/2))
    {
        Eigen::MatrixXd storeset_nz_rev(storeset_nz.rows(),storeset_nz.cols()); 
        storeset_nz_rev = storeset_nz.colwise().reverse();
        storeset_nz = storeset_nz_rev;
    }
    storesort.block(storesort_strt,0,storeset_nz.rows(),storeset_nz.cols()) = storeset_nz;  
    return storesort;
}

///////////////////////////////////////////////////////////

Eigen::MatrixXd NPAM::smoothened_traj_by_pts_skip(const Eigen::MatrixXd& storesort0x1, double space)
{
	// storing every n'th point to smoothen out the path 
    int count = 0;
    Eigen::MatrixXd store_spaced_pt = Eigen::MatrixXd::Constant(storesort0x1.rows(),storesort0x1.cols(),0); 
    store_spaced_pt.row(0) = storesort0x1.row(0);
    long int store_spaced_pt_idx = 0;
    int flagg = 0;
    for (long int i=1;i<storesort0x1.rows()-1;++i)
    {
        if (flagg==1)
        {
            flagg=0;
            continue;
        }
        if (std::abs(storesort0x1(i,1)-storesort0x1(i+1,1))<0.00001)
        {
            ++count;
            if (double(count)/space == round(count/space))
            {
                ++store_spaced_pt_idx;
                store_spaced_pt.row(store_spaced_pt_idx) = storesort0x1.row(i);
                count = 0;
            }
        }
        else
        {
            ++store_spaced_pt_idx;
            store_spaced_pt.row(store_spaced_pt_idx) = storesort0x1.row(i);
            ++store_spaced_pt_idx;
            store_spaced_pt.row(store_spaced_pt_idx) = storesort0x1.row(i+1);
            flagg = 1;  
        }
    }

    // adding the last point
    store_spaced_pt.row(store_spaced_pt_idx) = storesort0x1.row(storesort0x1.rows()-1);
    Eigen::MatrixXd storesort0x1tp = store_spaced_pt.block(0,0,store_spaced_pt_idx+1,store_spaced_pt.cols());
    return storesort0x1tp;
}

///////////////////////////////////////////////////////////

Eigen::MatrixXd NPAM::skip_toolpath_lines(const Eigen::MatrixXd& storesort0x1tp,int skip_lines)
{
    int dir1 = 1;
    int dir2 = 0;
    int flip = 0;
    int skip_lines_counter = 0;
    long int storesort_strt = 0;
    long int storeset_idx = 0;
    long int storesort_mid_pts_idx = 0;    
    Eigen::MatrixXd storeset = Eigen::MatrixXd::Constant(storesort0x1tp.rows(),storesort0x1tp.cols(),0);
    Eigen::MatrixXd storesort = Eigen::MatrixXd::Constant(storesort0x1tp.rows(),storesort0x1tp.cols(),0);
    Eigen::MatrixXd storesort_mid_pts = Eigen::MatrixXd::Constant(storesort0x1tp.rows(),storesort0x1tp.cols(),0);

    for (long i=0;i<storesort0x1tp.rows()-1;++i)
    {
        if (std::abs(storesort0x1tp(i,dir1)-storesort0x1tp(i+1,dir1))<0.00001)
        {
            // std::cout << "bug4..." << std::endl;
            storeset.row(storeset_idx) = storesort0x1tp.row(i);
            ++storeset_idx;
        }
        else
        {
            // std::cout << "bug47..." << std::endl;
            
            storeset.row(storeset_idx) = storesort0x1tp.row(i);
            Eigen::MatrixXd storeset_nz(storeset_idx+1,storeset.cols());
            storeset_nz = storeset.block(0,0,storeset_idx+1,storeset.cols());
            std::vector<std::vector<double> > storeset_nz_vec;
            storeset_nz_vec = ut::mat_to_vec(storeset_nz);
            storeset_nz_vec = ut::SortRows(storeset_nz_vec,dir2);
            storeset_nz = ut::vec_to_mat(storeset_nz_vec);
            if (skip_lines!=0)
            {
                if (storesort_strt!=0)
                {
                    if (skip_lines_counter<skip_lines)
                    {
                        skip_lines_counter++;
                        if (double(flip)/2==round(double(flip)/2))
                        {
                            storesort_mid_pts.row(storesort_mid_pts_idx++) = storeset_nz.row(storeset_nz.rows()-1);
                        }
                        else
                        {
                            storesort_mid_pts.row(storesort_mid_pts_idx++) = storeset_nz.row(0);    
                        }
                        storeset_idx = 0;
                        continue;                       
                    }
                    else
                    {
                        skip_lines_counter = 0;
                    }
                }
            }
            if (double(flip)/2==round(double(flip)/2))
            {
                Eigen::MatrixXd storeset_nz_rev(storeset_nz.rows(),storeset_nz.cols()); 
                storeset_nz_rev = storeset_nz.colwise().reverse();
                storeset_nz = storeset_nz_rev;
            }

            flip++;
            
            Eigen::MatrixXd storesort_mid_pts_filled = storesort_mid_pts.block(0,0,storesort_mid_pts_idx,storesort_mid_pts.cols());
            storesort.block(storesort_strt,0,storesort_mid_pts_filled.rows(),storesort_mid_pts_filled.cols()) = storesort_mid_pts_filled;
            storesort_strt = storesort_strt + storesort_mid_pts_idx;
            storesort.block(storesort_strt,0,storeset_nz.rows(),storeset_nz.cols()) = storeset_nz;
            storesort_strt = storesort_strt + storeset_idx+1;
            storeset_idx = 0; 
            storesort_mid_pts_idx = 0;           
        }
    }
    if (skip_lines_counter>=skip_lines)
    {
    	Eigen::MatrixXd storesort_mid_pts_filled = storesort_mid_pts.block(0,0,storesort_mid_pts_idx,storesort_mid_pts.cols());
        storesort.block(storesort_strt,0,storesort_mid_pts_filled.rows(),storesort_mid_pts_filled.cols()) = storesort_mid_pts_filled;
        storesort_strt = storesort_strt + storesort_mid_pts_idx;        
        storeset.row(storeset_idx) = storesort0x1tp.row(storesort0x1tp.rows()-1);
        Eigen::MatrixXd storeset_nz(storeset_idx+1,storeset.cols());
        storeset_nz = storeset.block(0,0,storeset_idx+1,storeset.cols());
        std::vector<std::vector<double> > storeset_nz_vec;
        storeset_nz_vec = ut::mat_to_vec(storeset_nz);
        storeset_nz_vec = ut::SortRows(storeset_nz_vec,dir2);
        storeset_nz = ut::vec_to_mat(storeset_nz_vec);
        // this is to get the direction of last line travel
        if (double(flip)/2==round(double(flip)/2))
        {
            Eigen::MatrixXd storeset_nz_rev(storeset_nz.rows(),storeset_nz.cols()); 
            storeset_nz_rev = storeset_nz.colwise().reverse();
            storeset_nz = storeset_nz_rev;
        }
        storesort.block(storesort_strt,0,storeset_nz.rows(),storeset_nz.cols()) = storeset_nz;  
        storesort_strt = storesort_strt + storeset_nz.rows();
    }
    Eigen::MatrixXd skipped_lines_pts = storesort.block(0,0,storesort_strt,storesort.cols());
    return skipped_lines_pts;
}

///////////////////////////////////////////////////////////

Eigen::MatrixXd NPAM::Infill_Path_with_Normals(const Eigen::MatrixXd& fillpts, bool FlipTravel, double space, double hatch_angle, double x_avg, double y_avg, int skip_lines)
{
    // function for creating path from projected points for tcp travel along x direction
    // INPUT: projected points on the surface (equally spaced along x and y axes)
    // OUTPUT: points arranged along 0 degree path with their Rx and Ry value

    // aligning points
    Eigen::MatrixXd storesort = NPAM::align_pts(fillpts);

    Eigen::MatrixXd storesort0x1(storesort.rows(),6);
    storesort0x1.block(0,0,storesort.rows(),3) = storesort.block(0,0,storesort.rows(),3);
    storesort0x1.block(0,3,storesort.rows(),3) = -storesort.block(0,3,storesort.rows(),3);

    // skipping pts along line to smoothen motion
    Eigen::MatrixXd storesort0x1tp = NPAM::smoothened_traj_by_pts_skip(storesort0x1, space);

    // skipping the toolpath lines
    Eigen::MatrixXd storesort0x1tp_reduced_lines = NPAM::skip_toolpath_lines(storesort0x1tp,skip_lines);

    // flip the direction of travel
    if (FlipTravel==1)
    {
        Eigen::MatrixXd storesort0x1tp_rev(storesort0x1tp_reduced_lines.rows(),storesort0x1tp_reduced_lines.cols());
        storesort0x1tp_rev = storesort0x1tp_reduced_lines.colwise().reverse();
        storesort0x1tp = storesort0x1tp_rev;
    }

    //apply hatching angle
    Eigen::MatrixXd storesort0x1tp_new = rotate_pts(storesort0x1tp_reduced_lines.block(0,0,storesort0x1tp_reduced_lines.rows(),2),hatch_angle,x_avg,y_avg);
    storesort0x1tp_reduced_lines.block(0,0,storesort0x1tp_reduced_lines.rows(),2) = storesort0x1tp_new;
    return storesort0x1tp_reduced_lines;
}

///////////////////////////////////////////////////////////

Eigen::MatrixXd NPAM::Infill_Path_with_bxbybz(const Eigen::MatrixXd& fillpts, bool FlipTravel, double space, double hatch_angle, double x_avg, double y_avg, int skip_lines)
{
    // function for creating path from projected points for tcp travel along x direction
    // INPUT: projected points on the surface (equally spaced along x and y axes)
    // OUTPUT: points arranged along 0 degree path with their Rx and Ry value

    // aligning points
    Eigen::MatrixXd storesort = NPAM::align_pts(fillpts);  

    Eigen::MatrixXd storesort0x1(storesort.rows(),12);
    storesort0x1.block(0,0,storesort.rows(),3) = storesort.block(0,0,storesort.rows(),3);
    Eigen::MatrixXd bxbybz = ut::compute_TCP(storesort.block(0,0,storesort.rows(),3),-storesort.block(0,3,storesort.rows(),3));
    storesort0x1.block(0,3,storesort.rows(),9) = bxbybz;

    // skipping pts along line to smoothen motion
    Eigen::MatrixXd storesort0x1tp = NPAM::smoothened_traj_by_pts_skip(storesort0x1, space);

    // skipping the toolpath lines
    Eigen::MatrixXd storesort0x1tp_reduced_lines = NPAM::skip_toolpath_lines(storesort0x1tp,skip_lines);

    // flip the direction of travel
    if (FlipTravel==1)
    {
        Eigen::MatrixXd storesort0x1tp_rev(storesort0x1tp_reduced_lines.rows(),storesort0x1tp_reduced_lines.cols());
        storesort0x1tp_rev = storesort0x1tp_reduced_lines.colwise().reverse();
        storesort0x1tp_reduced_lines = storesort0x1tp_rev;
    }

    //apply hatching angle
    Eigen::MatrixXd storesort0x1tp_new = rotate_pts(storesort0x1tp_reduced_lines.block(0,0,storesort0x1tp_reduced_lines.rows(),2),hatch_angle,x_avg,y_avg);
    storesort0x1tp_reduced_lines.block(0,0,storesort0x1tp_reduced_lines.rows(),2) = storesort0x1tp_new;
    return storesort0x1tp_reduced_lines;
}

///////////////////////////////////////////////////////////

Eigen::MatrixXd NPAM::Infill_Path_with_euler(const Eigen::MatrixXd& fillpts, bool FlipTravel, double space, double hatch_angle, double x_avg, double y_avg, int skip_lines)
{
    // function for creating path from projected points for tcp travel along x direction
    // INPUT: projected points on the surface (equally spaced along x and y axes)
    // OUTPUT: points arranged along 0 degree path with their Rx and Ry value

    // aligning points
    Eigen::MatrixXd storesort = NPAM::align_pts(fillpts);

    Eigen::MatrixXd storesort0x1(storesort.rows(),12);
    storesort0x1.block(0,0,storesort.rows(),3) = storesort.block(0,0,storesort.rows(),3);
    Eigen::MatrixXd bxbybz = ut::compute_TCP(storesort.block(0,0,storesort.rows(),3),-storesort.block(0,3,storesort.rows(),3));
    Eigen::MatrixXd cba = rtf::bxbybz2eul(bxbybz);
    storesort0x1.block(0,3,storesort.rows(),3) = cba;

    // skipping pts along line to smoothen motion
    Eigen::MatrixXd storesort0x1tp = NPAM::smoothened_traj_by_pts_skip(storesort0x1, space);

    // skipping the toolpath lines
    Eigen::MatrixXd storesort0x1tp_reduced_lines = NPAM::skip_toolpath_lines(storesort0x1tp,skip_lines);

    // flip the direction of travel
    if (FlipTravel==1)
    {
        Eigen::MatrixXd storesort0x1tp_rev(storesort0x1tp_reduced_lines.rows(),storesort0x1tp_reduced_lines.cols());
        storesort0x1tp_rev = storesort0x1tp_reduced_lines.colwise().reverse();
        storesort0x1tp = storesort0x1tp_rev;
    }

    //apply hatching angle
    Eigen::MatrixXd storesort0x1tp_new = rotate_pts(storesort0x1tp_reduced_lines.block(0,0,storesort0x1tp_reduced_lines.rows(),2),hatch_angle,x_avg,y_avg);
    storesort0x1tp_reduced_lines.block(0,0,storesort0x1tp_reduced_lines.rows(),2) = storesort0x1tp_new;
    return storesort0x1tp_reduced_lines;
}

///////////////////////////////////////////////////////////
