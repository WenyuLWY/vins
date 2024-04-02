    //DepthCleaner
    cv::Mat depth32f;
    filteredImage.convertTo(depth32f, CV_32F);
    auto cleaner = cv::rgbd::DepthCleaner::create(CV_32F, 5, cv::rgbd::DepthCleaner::DEPTH_CLEANER_NIL);
    cv::Mat cleanedDepth;

    timer = new Timer("DepthCleaner");
        cleaner->operator()(depth32f, cleanedDepth);
    delete timer;