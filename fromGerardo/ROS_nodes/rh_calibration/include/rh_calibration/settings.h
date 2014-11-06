//
//  settings.h (for opencv chessboard, legacy class)
//  Camera Calibration
//
//  Created by Gerardo Aragon on 21/11/2012.
//  Copyright (c) 2012 Gerardo Aragon. All rights reserved.

#ifndef SETTINGS_H
#define SETTINGS_H

class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern {CHESSBOARD, NOT_EXISTING};

    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Square_Size"]  >> squareSize;
        node["Max_Error"] >> maxError;
        node["Write_dirPathImages"] >> imageOutputDir;
        node["Debug_Mode"] >> debug_mode;
        node["Save_Mode"] >> save_mode;
        node["Save_Points_Mode"] >> save_points_mode;
        node["Write_outputFileNameLeftCamera"] >> outputFileNameL;
        node["Write_outputFileNameRightCamera"] >> outputFileNameR;
        node["Write_outputFileNameExtrinsic"] >> outputFileNameExtrinsic;
        node["Write_outputGripper2Camera"] >> outputG2C;
        node["Scale_Input"] >> scaleInput;
        node["Write_stereoCalib"] >> outputStereo;
        node["Write_outputFileNameEulerAngles"] >> outputEuler;
        node["Write_outputRobotG2CT"] >> outputRobotG2CT;
        patternToUse = "CHESSBOARD";
        if(debug_mode)
        {
            node["Input"] >> input;
            node["NoImages_Camera"] >> imgCal;
            node["NoImages_HandEye"] >> imgHE;
            ROS_WARN("Debug mode enable!");
        }
        else
        {
            imgCal = 0;
            imgHE = 0;
        }
        interprate();
    }
    void interprate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }

        if(maxError < 0)
        {
            maxError = 0;
        }

        if(scaleInput < 0)
        {
            scaleInput = 0;
        }

        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
        if (calibrationPattern == NOT_EXISTING)
        {
            cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
            goodInput = false;
        }

        if(debug_mode)
        {
            input = node_path + "/" + input;
            ROS_INFO_STREAM("Image list: " << input);
            if(!readStringList(input, imageList))
                ROS_ERROR("Image list is not a readable file");
            atImageList = 0;
        }

    }

    Mat nextImage()
    {
        ROS_INFO_STREAM("Size of image list: " << (int)imageList.size());
        int nextToLastInList = (int)imageList.size() - 2;
        Mat result;
        if( atImageList >= (int)imageList.size() )
        {
            atImageList = nextToLastInList;
        }

        ROS_INFO_STREAM("Reading: " << imageList[atImageList]);
        result = imread(imageList[atImageList++], 1);
        imageSize = result.size();

        return result;
    }

    static bool readStringList( const string& filename, vector<string>& l )
    {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
        {
            //cout << (string)*it;
            l.push_back((string)*it);
        }
        return true;
    }

public:
    Size boardSize;					// The size of the board -> Number of items by width and height
    Pattern calibrationPattern;		// One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;				// The size of a square in your defined unit (point, millimeter,etc).
    float maxError;                 // Maximum reprojection error allowed
    bool debug_mode;				// Write extrinsic parameters
    bool save_mode;					// Save processed images in hard disk
    bool save_points_mode;          // Save image points of the calibration target for each camera
    string outputFileNameL;			// The name of the file where to write the left camera parameters
    string outputFileNameR;			// The name of the file where to write the right camera parameters
    string imageOutputDir;			// Folder where to save images
    string outputFileNameExtrinsic;	// The name of the file where to write the extrinsic parameters
    string outputG2C;				// The name of the file where to write the transformations
    string outputStereo;			// The name of the file where to write the stereo calibration parameters
    string outputEuler;             // The name of the file where to write the euler angles of each camera
    string outputRobotG2CT;          // The name of the file where to write the the gripper to calibration target transformation
    int imgCal;                     // The number of images to be used in the calibration while in debug mode
    int imgHE;                      // The number of images to be used in the hand eye calibration while in debug mode

    bool goodInput;
    std::string node_path;
    int nrFrames;

    float scaleInput;				// Set if images should be resized

    string input;               	// The input (only for debug purposes)
    vector<string> imageList;
    int atImageList;

    Size imageSize;

private:
    string patternToUse;

};

void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

#endif
