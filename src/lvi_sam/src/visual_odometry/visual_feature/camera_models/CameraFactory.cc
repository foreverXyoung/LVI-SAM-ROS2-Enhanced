#include "CameraFactory.h"

#include <boost/algorithm/string.hpp>


#include "CataCamera.h"
#include "EquidistantCamera.h"
#include "PinholeCamera.h"
#include "ScaramuzzaCamera.h"


#include "ceres/ceres.h"

namespace camodocal
{

namespace
{
bool isUsableCamera(const CameraPtr& camera)
{
    if (!camera || camera->imageWidth() <= 0 || camera->imageHeight() <= 0)
        return false;

    Eigen::Vector3d ray;
    camera->liftProjective(
        Eigen::Vector2d(camera->imageWidth() * 0.5,
                        camera->imageHeight() * 0.5),
        ray);
    return ray.allFinite() && ray.norm() > 1e-9;
}
}

boost::shared_ptr<CameraFactory> CameraFactory::m_instance;

CameraFactory::CameraFactory()
{

}

boost::shared_ptr<CameraFactory>
CameraFactory::instance(void)
{
    if (m_instance.get() == 0)
    {
        m_instance.reset(new CameraFactory);
    }

    return m_instance;
}

CameraPtr
CameraFactory::generateCamera(Camera::ModelType modelType,
                              const std::string& cameraName,
                              cv::Size imageSize) const
{
    switch (modelType)
    {
    case Camera::KANNALA_BRANDT:
    {
        EquidistantCameraPtr camera(new EquidistantCamera);

        EquidistantCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::PINHOLE:
    {
        PinholeCameraPtr camera(new PinholeCamera);

        PinholeCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::SCARAMUZZA:
    {
        OCAMCameraPtr camera(new OCAMCamera);

        OCAMCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::MEI:
    default:
    {
        CataCameraPtr camera(new CataCamera);

        CataCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    }
}

CameraPtr
CameraFactory::generateCameraFromYamlFile(std::shared_ptr<rclcpp::Node> n, const std::string& filename)
{
    // cv::FileStorage fs(filename, cv::FileStorage::READ);

    // if (!fs.isOpened())
    // {
    //     rclcpp::shutdown();
    //     return CameraPtr();
    // }

    // std::shared_ptr<rclcpp::Node> node;
    Camera::ModelType modelType = Camera::MEI;
    n->declare_parameter("model_type", "mei");
    // if (modelType_temp != camodocal::Camera::ModelType::NONE)
    if(true)
    {
        std::string sModelType;
        // modelType_temp >> sModelType;
        n->get_parameter("model_type", sModelType);

        if (boost::iequals(sModelType, "kannala_brandt"))
        {
            modelType = Camera::KANNALA_BRANDT;
        }
        else if (boost::iequals(sModelType, "mei"))
        {
            modelType = Camera::MEI;
        }
        else if (boost::iequals(sModelType, "scaramuzza"))
        {
            modelType = Camera::SCARAMUZZA;
        }
        else if (boost::iequals(sModelType, "pinhole"))
        {
            modelType = Camera::PINHOLE;
        }
        else
        {
            std::cerr << "# ERROR: Unknown camera model: " << sModelType << std::endl;
            return CameraPtr();
        }
    }

    // ROS2 parameter overrides only become available after declaration.  The
    // original port called get_parameter() for these fields without ever
    // declaring them, leaving the camera size at zero on a default rclcpp
    // node.  Declare them once here for every camera model.
    if (!n->has_parameter("image_width"))
        n->declare_parameter<int>("image_width", 0);
    if (!n->has_parameter("image_height"))
        n->declare_parameter<int>("image_height", 0);

    switch (modelType)
    {
    case Camera::KANNALA_BRANDT:
    {
        EquidistantCameraPtr camera(new EquidistantCamera);

        EquidistantCamera::Parameters params = camera->getParameters();
        if (!params.readFromYamlFile(n, filename)) return CameraPtr();
        camera->setParameters(params);
        if (!isUsableCamera(camera)) return CameraPtr();
        return camera;
    }
    case Camera::PINHOLE:
    {
        PinholeCameraPtr camera(new PinholeCamera);

        PinholeCamera::Parameters params = camera->getParameters();
        if (!params.readFromYamlFile(n, filename)) return CameraPtr();
        camera->setParameters(params);
        if (!isUsableCamera(camera)) return CameraPtr();
        return camera;
    }
    case Camera::SCARAMUZZA:
    {
        OCAMCameraPtr camera(new OCAMCamera);

        OCAMCamera::Parameters params = camera->getParameters();
        if (!params.readFromYamlFile(n, filename)) return CameraPtr();
        camera->setParameters(params);
        if (!isUsableCamera(camera)) return CameraPtr();
        return camera;
    }
    case Camera::MEI:
    default:
    {
        CataCameraPtr camera(new CataCamera);

        CataCamera::Parameters params = camera->getParameters();
        if (!params.readFromYamlFile(n, filename)) return CameraPtr();
        camera->setParameters(params);
        if (!isUsableCamera(camera)) return CameraPtr();
        return camera;
    }
    }

    return CameraPtr();
}

}

