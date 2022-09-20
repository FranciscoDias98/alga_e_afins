#include "compressor.h"
#include "CompressedPointCloud.h"
#include "octree_pointcloud_compression_2.h"
#include <chrono>
#include <time.h>
#include <string>
#include<unistd.h>
// testes tempos
unsigned int x = 0;
unsigned long tempos = 0;
float size_compressed =0;
float size_original =0;

unsigned long tempos_test = 0;
float size_compressed_test =0;
float size_original_test =0;
float points_second = 0;

pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder1;




Alfa_Pc_Compress::Alfa_Pc_Compress()
{
    std::cout << " ---------- ALFA-Pc Compressor Constructor -----------" << std::endl;

    //-------------- SW-HW Memory Init ---------------------
   
    unsigned int region_size = 0x10000;
    off_t axi_pbase = 0xA0000000;
    u_int32_t *hw32_vptr;
    u64 *ddr_pointer;
    int fd;
    unsigned int ddr_size = 0x060000;
    off_t ddr_ptr_base = 0x0F000000; // physical base address
    //Map the physical address into user space getting a virtual address for it

    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) != -1) {
        ddr_pointer = (u64 *)mmap(NULL, ddr_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, ddr_ptr_base);
        hw32_vptr = (u_int32_t *)mmap(NULL, region_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, axi_pbase);
    }
    else
        ROS_INFO("NAO ENTROU NO NMAP :(");

    /*
    int16_t a16_points[4];
    a16_points[0] = 0x0201;
    a16_points[1] = 0x0103;
    a16_points[2] = 0x0302;
    a16_points[3] = 0x0201;
    memcpy((void*)(ddr_pointer), a16_points,sizeof(int32_t)*2);
    a16_points[0] = 0x0103;
    a16_points[1] = 0x0302;
    a16_points[2] = 0x0201;
    a16_points[3] = 0x0103;
    memcpy((void*)(ddr_pointer+1),a16_points,sizeof(int16_t)*4);
    a16_points[0] = 0x0302;
    a16_points[1] = 0x0000;
    a16_points[2] = 0x0000;
    a16_points[3] = 0x0000;
    memcpy((void*)(ddr_pointer+2),a16_points,sizeof(int16_t)*4);
    a16_points[0] = 0x0000;
    a16_points[1] = 0x0000;
    a16_points[2] = 0x0000;
    a16_points[3] = 0x0000;
    memcpy((void*)(ddr_pointer+3),a16_points,sizeof(int16_t)*4);

    sleep(1);

    vector<uint32_t> two_matrix;
    two_matrix.push_back(1);
    // two_matrix.push_back(0x02030102);
    // two_matrix.push_back(0x03010203);
    // two_matrix.push_back(0x01020301);
    // two_matrix.push_back(0x02030000);
    //Write in Hw
    write_hardware_registers(two_matrix, hw32_vptr);

    //Read in Hw

    while(!hw32_vptr[1]){
      ROS_INFO("WAITING");
    }
    int32_t array[2];
    for(int i=0; i<5; i++){
      memcpy((void*)(array), ddr_pointer+i,sizeof(int16_t)*4);
      printf("%X\n", array[0]);
      printf("%X\n", array[1]);
    }

    */
    //--------------------------------------------------------//

    in_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    out_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    cluster1.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cluster2.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    compressed_data_vector.clear();

    set_compression_profile(); // define compression profile
    spin();






}

void Alfa_Pc_Compress::set_compression_profile()
{
    std::cout << "Setting Compression Profile" << std::endl;


    compression_profile = pcl::io::MANUAL_CONFIGURATION;

    show_statistics = true;
    compression_profile_.pointResolution = 0.01; //
    compression_profile_.octreeResolution = 0.03; //-----> ALTERAR NESTE!!!!!!!!! voxel size in cubic meters (1m is 0.01 cm)
    compression_profile_.doVoxelGridDownSampling = true;
    compression_profile_.iFrameRate = 10; // number of prediction frames
    compression_profile_.colorBitResolution = 0;
    compression_profile_.doColorEncoding = false;

    PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,
                        show_statistics,
                        compression_profile_.pointResolution,
                        compression_profile_.octreeResolution,
                        compression_profile_.doVoxelGridDownSampling,
                        compression_profile_.iFrameRate,
                        compression_profile_.doColorEncoding,
                        compression_profile_.colorBitResolution);


    output_metrics.message_tag = "Compression performance";
}

void Alfa_Pc_Compress::process_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud, const sensor_msgs::PointCloud2ConstPtr& header)
{
    output_metrics.metrics.clear();

    std::stringstream compressed_data;
    std::stringstream compressed_data2;

    ROS_INFO("Compressing cloud with frame [%s]\n", output_cloud->header.frame_id.c_str());
    ROS_INFO("Compressing cloud with frame [%d]\n", output_cloud->header.seq);


    // compress
    auto start = high_resolution_clock::now();
    //do_Compression(output_cloud);
    PointCloudEncoder->encodePointCloud_2(output_cloud,compressed_data);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    std::cout << "Fiz Compressao" << std::endl;

    size_original_test = size_original_test + (static_cast<float> (output_cloud->points.size()) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f)*1000;
    x++;
    compressed_data.seekg(0,ios::end);
    size_compressed_test = size_compressed_test+compressed_data.tellg();
    tempos_test = tempos_test + duration.count();

    points_second += 1000*output_cloud->points.size() / duration.count();

    if(x==100){
        x=0;
        exe_time();
    }

    points_second += 1000*output_cloud->points.size() / duration.count();


    ROS_INFO("Compressing in %ld ms",duration.count());

    // push metrics to monitor
    metrics(compressed_data,output_cloud,duration.count());


    // create object to hold the compressed data
    //compressed_pointcloud_transport::CompressedPointCloud output_compressed;
    output_compressed.header = header->header;
    output_compressed.data = compressed_data.str();




    // pub compressed data
    publish_pointcloud(output_compressed);
    publish_metrics(output_metrics);
}



void Alfa_Pc_Compress::run_worker(int thread_number)
{
    std::cout << "THREAD: " << thread_number << std::endl;
    std::cout << "Cloud Size: " << pcloud->size()<< std::endl;

    std::stringstream compressed_data;
    std::stringstream compressed_data1;

    for(int i = (pcloud->size()/number_threads)*thread_number;i<= (pcloud->size()/number_threads)*(thread_number+1);i++)
    {
        pcl::PointXYZRGB point = (*pcloud)[i];
//        mutex_cluster.lock();
//        //push point to cluster
        if(thread_number == 0)
        {
            mutex_cluster.lock();
            cluster1->push_back(point);
            mutex_cluster.unlock();


        }else if (thread_number == 1)
        {
            mutex_cluster.lock();
            cluster2->push_back(point);
            mutex_cluster.unlock();
        }

//        mutex_cluster.unlock();
    }

//    // vai ter de ser array de encoders
//    //PointCloudEncoder->encodePointCloud(clusters[thread_number],compressed_data_vector[thread_number]);
//    std::cout << "Fiz cluster: " << std::endl;

//    mutex_encoder.lock();
//    encoder_vector[thread_number]->encodePointCloud_2(clusters[thread_number],compressed_data_vector[thread_number]);
//    mutex_encoder.unlock();

//    mutex_compressed_data.lock();
//    output_compressed.data += compressed_data_vector[thread_number].str();
//    mutex_compressed_data.unlock();

    if(thread_number == 0)
    {
        PointCloudEncoder->encodePointCloud_2(cluster1,compressed_data);
        std::cout << "Fiz compressao encoder 0" << std::endl;

        mutex_compressed_data.lock();
        output_compressed.data += compressed_data.str();
        mutex_compressed_data.unlock();

    }
    if(thread_number == 1)
    {

        PointCloudEncoder1->encodePointCloud_2(cluster2,compressed_data1);
        std::cout << "Fiz compressao encoder 1" << std::endl;

        mutex_compressed_data.lock();
        output_compressed.data += compressed_data1.str();
        mutex_compressed_data.unlock();

    }


}





alfa_msg::AlfaConfigure::Response Alfa_Pc_Compress::process_config(alfa_msg::AlfaConfigure::Request &req)
{
    update_compressionSettings(req);
    alfa_msg::AlfaConfigure::Response response;
    response.return_status = 1;
    return response;

}




void Alfa_Pc_Compress::metrics(std::stringstream& compressed_data, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud, double duration)
{
    compressed_data.seekg(0,ios::end);
    size_compressed = compressed_data.tellg();


    size_original = (static_cast<float> (output_cloud->points.size()) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f)*1000;


    ROS_INFO("Tree depth: %d\n",PointCloudEncoder->getTreeDepth());



    // alfa metrics
    alfa_msg::MetricMessage new_message;

    new_message.metric = size_original/1000;
    new_message.units = "kB";
    new_message.metric_name = "Original Size";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = output_cloud->size();
    new_message.units = "";
    new_message.metric_name = "Nº of Points in Point Cloud";
    output_metrics.metrics.push_back(new_message);


    new_message.metric = duration;
    new_message.units = "ms";
    new_message.metric_name = "Total processing time";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = size_compressed/1000;
    new_message.units = "kB";
    new_message.metric_name = "Compressed Size";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = size_original/size_compressed;
    new_message.units = "";
    new_message.metric_name = "Compression Ratio";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = PointCloudEncoder->getTreeDepth();
    new_message.units = "";
    new_message.metric_name = "Octree Depth";
    output_metrics.metrics.push_back(new_message);



}

void Alfa_Pc_Compress::update_compressionSettings(const alfa_msg::AlfaConfigure::Request configs)
{
    //compression_profile_.octreeResolution = configs.configurations[0].config;
    PointCloudEncoder->setResolution(configs.configurations[0].config);
    //delete(PointCloudEncoder);
}




void Alfa_Pc_Compress::exe_time()
{
    tempos_test = tempos_test/100 ;
    size_compressed_test = size_compressed_test/100;
    size_original_test = (size_original_test)/100;
    points_second = points_second/100;
    //std::ofstream myFile("./output/exe_time");
    //myFile<< "Exe. Time: "<< tempos_test << std::endl << "Point Cloud Size: "<< size_original_test << std::endl << "Compressed Size: "<<size_compressed_test<< std::endl << "Ratio: " << size_original_test/size_compressed_test << std::endl ;
    //myFile.close();
    ROS_INFO("-----------Acabei------------------------------------------------------------- \n");
    ROS_INFO("Time: %d\n", tempos_test);
    ROS_INFO("Point Cloud Size: %f\n", size_original_test);
    ROS_INFO("Compressed Size: %f\n", size_compressed_test);
    ROS_INFO("Ratio: %f\n", size_original_test/size_compressed_test);
    ROS_INFO("Points/s: %f\n", points_second);

    x=0;
    size_compressed_test = 0;
    size_original_test = 0;
    points_second = 0;
}


void Alfa_Pc_Compress::do_Compression(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud)
{

    using namespace std::chrono;

    std::stringstream compressed_data_;

    ROS_INFO("Compressing cloud with frame [%s]", in_cloud->header.frame_id.c_str());
    this->in_cloud = in_cloud;

    auto start = high_resolution_clock::now();

    //compress point cloud
    //point_cloud_encoder->setInputCloud(in_cloud);
    point_cloud_encoder->encodePointCloud(in_cloud,compressed_data_);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    ROS_INFO("Compressing in %ld ms",duration.count());
    ROS_INFO("Tree depth: %d\n",point_cloud_encoder->getTreeDepth());

    // testes tempos
    tempos = tempos + duration.count();
    compressed_data_.seekg(0,ios::end);
    size_compressed = size_compressed + compressed_data_.tellg();
    size_original = size_original + static_cast<float> (in_cloud->size()) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f;
    x++;
    //test
    if(x==100){
      exe_time();
    }
}
//void Alfa_Pc_Compress::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input_cloud)
//{

//    if((input_cloud->width * input_cloud->height) == 0)
//        return; //return if the cloud is not dense

//    // convert from sensor_msg::PointCloud2 to pcl::PointCloud<PointXYZI> for the encoder
//    try
//    {
//        pcl::fromROSMsg(*input_cloud, *in_cloud);
//    }
//    catch (std::runtime_error e)
//    {
//        ROS_ERROR_STREAM("Error in converting ros cloud to pcl cloud: " << e.what());
//    }

//    //do_Compression(in_cloud);
//    point_cloud_encoder->encodePointCloud(in_cloud,compressed_data);

//    compressed_pointcloud_transport::CompressedPointCloud output_compressed;
//    output_compressed.header= input_cloud->header;
//    output_compressed.data = compressed_data.str();

//    // Publish the data.
//    std::cout << "Publishing data" << std::endl;
//    pub.publish(output_compressed); // --------- > Nao publica direito Pq ?????
//    std::cout << "Data published" << std::endl;

//}



//void Alfa_Pc_Compress::process_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud, const sensor_msgs::PointCloud2ConstPtr& header)
//{
//    output_metrics.metrics.clear();
//    mutex_cluster.lock();
//    cluster1->clear();
//    cluster2->clear();
//    mutex_cluster.unlock();

//    if(multi_thread)
//    {
//        if(thread_list.size()>1)
//        {
//            for (int i =0;i < thread_list.size();i++)
//            {
//                thread_list[i]->join();
//            }

//            thread_list.clear();
//        }

//        if (number_threads >1)
//        {
//            thread_list.clear();
//            for (int i =0;i < number_threads;i++)
//            {
//                thread_list.push_back(new boost::thread(&Alfa_Pc_Compress::run_worker, this,i));
//            }
//            for (int i =0;i < number_threads;i++)
//            {
//                thread_list[i]->join();
//            }
//        }

//    }else{

//    std::stringstream compressed_data;
//    std::stringstream compressed_data2;

//    ROS_INFO("Compressing cloud with frame [%s]\n", output_cloud->header.frame_id.c_str());
//    ROS_INFO("Compressing cloud with frame [%d]\n", output_cloud->header.seq);


//    // compress
//    auto start = high_resolution_clock::now();
//    //do_Compression(output_cloud);
//    PointCloudEncoder->encodePointCloud_2(output_cloud,compressed_data);
//    auto stop = high_resolution_clock::now();
//    auto duration = duration_cast<milliseconds>(stop - start);
//    std::cout << "Fiz Compressao" << std::endl;


//    ROS_INFO("Compressing in %ld ms",duration.count());

//    // push metrics to monitor
//    metrics(compressed_data,output_cloud,duration.count());


//    // create object to hold the compressed data
//    //compressed_pointcloud_transport::CompressedPointCloud output_compressed;
//    output_compressed.header = header->header;
//    output_compressed.data = compressed_data.str();

//    }
//    // pub compressed data
//    //metrics(compressed_data,output_cloud,duration.count());
//    output_compressed.header = header->header;
//    publish_pointcloud(output_compressed);
//    publish_metrics(output_metrics);
//}

