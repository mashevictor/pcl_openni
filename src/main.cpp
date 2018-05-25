#include <iostream> 
#include <OpenNI.h>
#include <pcl/common/common_headers.h>       // for pcl::PointCloud
#include <pcl/visualization/pcl_visualizer.h>

openni::Device mDevice;
openni::VideoStream mColorStream;
openni::VideoStream mDepthStream;

bool init(){
    // Initial OpenNI
    if(openni::OpenNI::initialize() != openni::STATUS_OK){
        std::cerr << "OpenNI Initial Error: "  << openni::OpenNI::getExtendedError() << std::endl;
        return false;
    }
    // Open Device
    if(mDevice.open(openni::ANY_DEVICE) != openni::STATUS_OK) {
        std::cerr << "Can't Open Device: "  << openni::OpenNI::getExtendedError() << std::endl;
        return false;
    }
    return true;
}

bool createColorStream() {
    if(mDevice.hasSensor(openni::SENSOR_COLOR)) {
        if(mColorStream.create(mDevice, openni::SENSOR_COLOR) == openni::STATUS_OK) {
            // set video mode
            openni::VideoMode mMode;
            mMode.setResolution(640, 480);
            mMode.setFps(30);
            mMode.setPixelFormat( openni::PIXEL_FORMAT_RGB888 );

            if(mColorStream.setVideoMode(mMode) != openni::STATUS_OK) {
                std::cout << "Can't apply VideoMode: "  << openni::OpenNI::getExtendedError() << std::endl;
                return false;
            }
        } else {
            std::cerr << "Can't create color stream on device: " << openni::OpenNI::getExtendedError() << std::endl;
            return false;
        }

        // start color stream
        mColorStream.start();
        return true;
    }
    return false;
}

bool createDepthStream(){
    if(mDevice.hasSensor(openni::SENSOR_DEPTH)) {
        if(mDepthStream.create(mDevice, openni::SENSOR_DEPTH) == openni::STATUS_OK) {
            // set video mode
            openni::VideoMode mMode;
            mMode.setResolution(640,480);
            mMode.setFps(30);
            mMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);

            if(mDepthStream.setVideoMode(mMode) != openni::STATUS_OK) {
                std::cout << "Can't apply VideoMode to depth stream: " << openni::OpenNI::getExtendedError() << std::endl;
                return false;
            }
        } else {
            std::cerr << "Can't create depth stream on device: " << openni::OpenNI::getExtendedError() << std::endl;
            return false;
        }
        // start depth stream
        mDepthStream.start();
        // image registration
    if( mDevice.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) )
            mDevice.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    else
        std::cerr << "Don't support registration" << std::endl;
        return true;
    } else {
        std::cerr << "ERROR: This device does not have depth sensor" << std::endl;
        return false;
    }
}

//openni图像流转化成点云
bool getCloudXYZCoordinate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZRGB)  {
    openni::VideoFrameRef  colorFrame;
    mColorStream.readFrame(&colorFrame);
    openni::RGB888Pixel *pColor = (openni::RGB888Pixel*)colorFrame.getData();

    openni::VideoFrameRef  mDepthFrame;
    if(mDepthStream.readFrame(&mDepthFrame) == openni::STATUS_OK) {
        float fx,fy,fz;
        int i=0;
        //以米为单位
        double fScale = 0.001;
        openni::DepthPixel *pDepthArray = (openni::DepthPixel*)mDepthFrame.getData();
        for(int y = 0; y < mDepthFrame.getHeight(); y++) {
            for(int x = 0; x < mDepthFrame.getWidth(); x++) {
                int idx = x + y*mDepthFrame.getWidth();
                const openni::DepthPixel rDepth = pDepthArray[idx];
                openni::CoordinateConverter::convertDepthToWorld(mDepthStream,x,y,rDepth,&fx,&fy,&fz);
                fx = -fx;
                fy = -fy;
                cloud_XYZRGB->points[i].x = fx * fScale;
                cloud_XYZRGB->points[i].y = fy * fScale;
                cloud_XYZRGB->points[i].z = fz * fScale;
                cloud_XYZRGB->points[i].r = pColor[i].r;
                cloud_XYZRGB->points[i].g = pColor[i].g;
                cloud_XYZRGB->points[i].b = pColor[i].b;
                i++;
            }
        }
        return true;
    } else {
        std::cout << "getCloudXYZCoordinate: fail to read frame from depth stream" << std::endl;
        return false;
    }
}

int main(){ 
    //openni初始化、打开摄像头
    if(!init()) {
        std::cout << "Fail to init ..." << std::endl;
        return -1;
    }
    //openni创建图像流
    if(createColorStream() && createDepthStream())
        std::cout << "displayPointCloud: create color stream and depth stream ..." << std::endl;
    else{
        std::cout << "displayPointCloud: can not create color stream and depth stream ..." << std::endl;
        return -1;
    }
    //创建pcl云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_XYZRGB->width = 640;
    cloud_XYZRGB->height = 480;
    cloud_XYZRGB->points.resize(cloud_XYZRGB->width*cloud_XYZRGB->height);
    //pcl可视化
    pcl::visualization::PCLVisualizer::Ptr m_pViewer(new pcl::visualization::PCLVisualizer("Viewer"));
    m_pViewer->setCameraPosition(0, 0, -2, 0,-1, 0, 0);
    m_pViewer->addCoordinateSystem(0.3);
    while(!m_pViewer->wasStopped()) {
        getCloudXYZCoordinate(cloud_XYZRGB);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_XYZRGB);
        m_pViewer->addPointCloud<pcl::PointXYZRGB>(cloud_XYZRGB,rgb,"cloud");
        m_pViewer->spinOnce();
        m_pViewer->removeAllPointClouds();
    }
    mColorStream.destroy();
    mDepthStream.destroy();
    mDevice.close();
    openni::OpenNI::shutdown();

    return 0;
}
