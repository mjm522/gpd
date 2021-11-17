#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;


class GPDPy{

    private:
        std::string cfg_file;
        Eigen::Vector3d view_point_;
        gpd::GraspDetector* grasp_detector_;

        ////FUNCTION 1
        void FillMatrixXf(Eigen::MatrixXf& matrix, uint index, const gpd::candidate::Hand& hand){
            Eigen::Vector3d position = hand.getPosition();
            Eigen::Vector3d appraoch = hand.getApproach();
            Eigen::Vector3d binormal = hand.getBinormal();
            Eigen::Vector3d axis = hand.getAxis();

            matrix(index, 0) = position(0);
            matrix(index, 1) = position(1);
            matrix(index, 2) = position(2);
            matrix(index, 3) = appraoch(0);
            matrix(index, 4) = appraoch(1);
            matrix(index, 5) = appraoch(2);
            matrix(index, 6) = binormal(0);
            matrix(index, 7) = binormal(1);
            matrix(index, 8) = binormal(2);
            matrix(index, 9) = axis(0);
            matrix(index, 10) = axis(1);
            matrix(index, 11) = axis(2);
            matrix(index, 12) = hand.getGraspWidth();
            matrix(index, 13) = hand.getScore();
        }

    public:
        GPDPy(const std::string config_file){
            cfg_file = config_file;
            view_point_ = Eigen::Vector3d(0.0, 0.0, 0.0);
            grasp_detector_ = new gpd::GraspDetector(cfg_file);
        }
        ~GPDPy() {}

        ////FUNCTION 2
        Eigen::MatrixXf detectGrasps(const Eigen::MatrixXf source_cloud){

            printf("\n Input cloud size %d %d ", source_cloud.rows(), source_cloud.cols());

            PointCloudRGBA::Ptr cloud(new PointCloudRGBA);  

            cloud->points.resize(source_cloud.rows());
            for(int i=0; i<source_cloud.rows(); i++){
                cloud->points[i].getVector3fMap() = Eigen::Vector3d(source_cloud(i, 0), source_cloud(i, 1), source_cloud(i, 2)).cast<float>();
            }

            Eigen::Matrix3Xd view_points(3,1);
            view_points.col(0) = view_point_;

            // 1. Initialize cloud camera and set point cloud.
            gpd::util::Cloud* cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);

            // 2. Preprocess the point cloud.
            grasp_detector_->preprocessPointCloud(*cloud_camera_);

            // 3. Detect grasps in the point cloud.
            std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_);

            if (grasps.size() > 0){

                printf("\n Grasps detected!");

            }else{
                printf("\n No grasps detected!");
            }

            Eigen::MatrixXf graspsData(grasps.size(), 14);

            for (uint i = 0; i < grasps.size(); i++) {

                FillMatrixXf(graspsData, i, *grasps[i]);

            }
            return graspsData;
        }

};


