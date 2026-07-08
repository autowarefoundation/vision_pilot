#include <iostream>
#include <stdexcept>
#include <fstream>
#include <vehicle_interface/file_interface.hpp>

FileInterface::FileInterface(const std::string& filename)
{
    std::ifstream file(filename);
    if (file.is_open())
    {
        std::string line;
        while (std::getline(file, line))
        {
            // skip empty lines
            if (line.find_first_not_of(" \t\r\n") == std::string::npos)
            {
                continue;
            }
            try
            {
                double value = std::stod(line);
                speeds_.push_back(value);
            }
            catch (const std::exception& e)
            {
                std::cerr << "Warning: skipping invalid line: \"" << line
                    << "\" (" << e.what() << ")" << std::endl;
            }
        }

        file.close();
    }
}

double FileInterface::read()
{
    if (speeds_.empty())
    {
        throw std::runtime_error("FileInterface: no speeds loaded");
    }

    if (frame_cnt_ >= speeds_.size())
    {
        throw std::runtime_error("FileInterface: read() called past end of speeds data");
    }

    return speeds_[frame_cnt_++];
}

void FileInterface::write(double steering, double acceleration)
{
}
