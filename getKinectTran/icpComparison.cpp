cerdogan@SchunkArm:~/Research/toolsLWA4/getKinectTran/build$ ./testICP 
has converged:1 score: 0.0150163
 -0.461293   0.874387   0.150525    -0.1843
  0.769916   0.478803  -0.421876   0.688166
 -0.440954 -0.0787167  -0.894071    1.48068
         0          0          0          1
[DEBUG] Orig point: -0.161206 0.509984 0.548732 , Tf:  0.10722 0.448766 0.668524 error: 0.30025
[DEBUG] Orig point: -0.271806 0.379802 0.422296 , Tf:  0.11305  0.24785 0.597656 error: 0.443031
[DEBUG] Orig point: -0.271774 0.226677 0.536259 , Tf: -0.0440712 0.199391 0.645519 error: 0.254029
[DEBUG] Orig point: -0.272123 -3.7e-05   0.4727 , Tf: -0.159043 0.0499155  0.530041 error: 0.136273
[DEBUG] Orig point: -0.13066 0.334796 0.468255 , Tf: 0.0315305 0.335866 0.539045 error: 0.176969
[DEBUG] Orig point: 0.024201 0.415507 0.513811 , Tf: 0.0131715 0.511785 0.497685 error: 0.0982402
[DEBUG] Orig point:  0.10084 0.341369 0.659931 , Tf: -0.144265 0.571494 0.550379 error: 0.353603
cerdogan@SchunkArm:~/Research/toolsLWA4/getKinectTran/build$ make
[ 25%] Built target getKinectTran
[ 50%] Built target moveEE_predef
[ 75%] Built target testCalib
Scanning dependencies of target testICP
[100%] Building CXX object CMakeFiles/testICP.dir/testICP.cpp.o
/home/cerdogan/Research/toolsLWA4/getKinectTran/testICP.cpp: In function ‘int main(int, char**)’:
/home/cerdogan/Research/toolsLWA4/getKinectTran/testICP.cpp:83:29: warning: ‘void pcl::Registration<PointSource, PointTarget, Scalar>::setInputCloud(const PointCloudSourceConstPtr&) [with PointSource = pcl::PointXYZ; PointTarget = pcl::PointXYZ; Scalar = float; pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSourceConstPtr = boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >]’ is deprecated (declared at /usr/local/include/pcl-1.7/pcl/registration/impl/registration.hpp:43): [pcl::registration::Registration::setInputCloud] setInputCloud is deprecated. Please use setInputSource instead. [-Wdeprecated-declarations]
Linking CXX executable testICP
[100%] Built target testICP
cerdogan@SchunkArm:~/Research/toolsLWA4/getKinectTran/build$ ./testICP 
has converged:1 score: 0.000288113
 0.975264 0.0787744  0.206533 -0.390849
0.00822639  0.920764 -0.390034  0.650055
 0.220893 -0.382086 -0.897341   1.41051
        0         0         0         1
[DEBUG] Orig point: -0.161206 0.509984 0.548732 , Tf: -0.156516 0.498685 0.559594 error: 0.0163601
[DEBUG] Orig point: -0.271806 0.379802 0.422296 , Tf: -0.255488 0.360787 0.430706 error: 0.0264311
[DEBUG] Orig point: -0.271774 0.226677 0.536259 , Tf: -0.264219 0.227512 0.537888 error: 0.00777369
[DEBUG] Orig point: -0.272123 -3.7e-05   0.4727 , Tf: -0.257138 0.0192125   0.46401 error: 0.0258959
[DEBUG] Orig point: -0.13066 0.334796 0.468255 , Tf: -0.128832 0.338115 0.466369 error: 0.004233
[DEBUG] Orig point: 0.024201 0.415507 0.513811 , Tf: 0.0266722 0.420824 0.510787 error: 0.00659691
[DEBUG] Orig point:  0.10084 0.341369 0.659931 , Tf: 0.0909265 0.354803 0.661255 error: 0.0167484
[DEBUG] Orig point: -0.015906 0.348108 0.594523 , Tf: -0.0208514 0.358138 0.589502 error: 0.0122584
[DEBUG] Orig point: 0.040345 0.103321  0.45221 , Tf: 0.0297232 0.109691 0.443903 error: 0.0149134
[DEBUG] Orig point: 0.050239 0.124578 0.451264 , Tf: 0.0380571 0.130135  0.44585 error: 0.0144429
[DEBUG] Orig point: 0.045849  0.31044 0.446621 , Tf: 0.0341189 0.299646 0.451501 error: 0.0166709
[DEBUG] Orig point: 0.050178 0.468402 0.288904 , Tf: 0.0517228 0.445398 0.294134 error: 0.0236417

