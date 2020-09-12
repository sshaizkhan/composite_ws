# Custom message for different types waypoints broadcasting

# getting x-y-z-bx-by-bz, where x,y,z are poistion value and bx,by,bz are the orientation vectors  
gen_utilities/XYZ_bxbybz[] xyz_bxbybz

# getting x-y-z-c-b-a, where x,y,z are poistion value and c,b,a are the Euler angles  
gen_utilities/XYZ_cba[] xyz_cba

# getting x-y-z-nx-ny-nz, where x,y,z are poistion value and nx,ny,nz are the surface normals associated with the position  
gen_utilities/XYZ_nxnynz[] xyz_nxnynz

# group indices if any
gen_utilities/IdxGroup[] group_idx
int32[] status
