#ifndef NAV_TEXTPLOT
#define NAV_TEXTPLOT

#include "../nav.h"
#include "env/offsets.h"
#include "env/wind.h"
#include <boost/filesystem.hpp>

namespace nav {

    class motion;

// CLASS TEXTPLOT
// ==============
// ==============

class NAV_API textplot {
private:
    /**< weak pointer to motion class */
    nav::motion* _Pm;
    /**< conversion factor between radians and degrees */
    double _r2d;
public:
    /**< default constructor */
    textplot() = delete;
    /**< constructor based on reference to motion class */
    explicit textplot(nav::motion& Omotion);
    /**< copy constructor */
    textplot(const textplot&) = delete;
    /**< move constructor */
    textplot(textplot&&) = delete;
    /**< destructor */
    ~textplot();
    /**< copy assignment */
    textplot& operator=(const textplot&) = delete;
    /**< move assignment */
    textplot& operator=(textplot&&) = delete;

    /**< returns a string containing the name of the folder where the text files shall be stored. Also ensures
     * that the folder is created if it does not exist and that it is emptied if it had previous files. */
    std::string obtain_folder(const int& case_guid, const int& seed_order, const double& turb_factor,
                             const env::logic::OFFSETS_ID& offsets_id, const env::logic::WIND_ID& wind_id, const double& t_sec_gpsloss) const;
    /**< execute all class methods, storing the text files in the input folder */
    void execute(const std::string& st_folder) const;
    
    /**< write on console the final size of all vectors */
    void console_final_sizes(std::ostream& Ostream) const;
    /**< write on console the final state of the aircraft */
    void console_final_state(std::ostream& Ostream) const;
    /**< write on console the statistics of the control errors */
    void console_errors_control(std::ostream& Ostream) const;
    /**< write on console the air data filter errors */
    void console_errors_filter_air(std::ostream& Ostream) const;
    /**< write on console the attitude filter errors */
    void console_errors_filter_att(std::ostream& Ostream) const;
    /**< write on console the position filter errors */
    void console_errors_filter_pos(std::ostream& Ostream) const;

    /**< create a text file with results */
    void text_results(const boost::filesystem::path& path_folder) const;
    /**< create a text file with longitudinal control information for later plotting in MatLab */
    void text_control_long(const boost::filesystem::path& path_folder) const;
    /**< create a text file with lateral control information for later plotting in MatLab */
    void text_control_lat(const boost::filesystem::path& path_folder) const;
    /**< create a text file with inertial sensor information for later plotting in Matlab */
    void text_sensors_inertial(const boost::filesystem::path& path_folder) const;
    /**< create a text file with non inertial sensor information for later plotting in Matlab */
    void text_sensors_other(const boost::filesystem::path& path_folder) const;
    /**< create text files with filter results for later plotting in Matlab */
    void text_filter(const boost::filesystem::path& path_folder) const;
    /**< create a text file with position output results for later plotting in Matlab */
    void text_output_pos(const boost::filesystem::path& path_folder) const;
    /**< create a text file with description of the inputs and the outputs */
    void text_description(const boost::filesystem::path& path_folder) const;

    /**< create a series of text files for the image generation process (TO BE MODIFIED) */
    void text_images(const boost::filesystem::path& path_folder) const;
    /**< read text files created in text_images method and integrate trajectory (custom made for Michael) */
    static void read_text_images(unsigned long nel_lines_sens, unsigned long nel_lines_images);
}; // closes class textplot

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace nav

#endif
