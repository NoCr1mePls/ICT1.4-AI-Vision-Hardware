#pragma once

/// @brief Structure to hold information about found trash
struct trash{
    int type; //< Trash classification
    int confidence; //< Confidence level of the classification
    float latitude; //< Latitude of the trash location
    float longitude; //< Longitude of the trash location
};