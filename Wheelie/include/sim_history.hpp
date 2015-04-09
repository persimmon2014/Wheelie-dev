#ifndef SIM_HISTORY_HPP
#define SIM_HISTORY_HPP

#include <roadie/hwm_network.hpp>
#include <hybrid-sim.hpp>
#include <roadie/osm_network.hpp>

#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>
#include <sstream>
#include <map>
#include <iomanip>
#include <vector>
#include <iostream>
#include <sstream>



struct _time
{
  int day;
  int hour;
  int minute;

  _time():day(0), hour(0), minute(0){}
};


/**************************************************
 * Structure for simulation history
 **************************************************/
struct _cell
{
  float avg_v;
  float den;
};

struct _stat
{
  std::vector<_cell> cells;
  float cell_len;
};

struct _sim_hist_micro
{
public:
  unsigned long 			                 starting_time; /*CURRENTLY MANUALLY SET IN MTSim.cpp */
  float					                       dt_sec;
  typedef float				                 avg;
  typedef float				                 den;
  typedef std::map<std::string, _stat> moment;
  std::vector<moment>			             history;

};




struct _sim_hist
{
public:
  unsigned long 			                 starting_time; /*CURRENTLY MANUALLY SET IN MTSim.cpp */
  float					                       dt_sec;
  typedef float				                 avg;
  typedef float				                 den;
  typedef std::map<std::string, _stat> moment;
  std::vector<moment>			             history;


public:
  moment get_state(unsigned long timestamp)
  {
    ulong         micro_per_sec  = (ulong)1e6;
    unsigned long	diff	         = timestamp - starting_time;
    diff			                  /= micro_per_sec;
    unsigned long	moment_ndx     = (unsigned long) std::floor(diff / dt_sec);

    if (moment_ndx > history.size())
    {
      std::cerr << "Moment yet to come: simulator hasn't kept up with reality." << std::endl;
      return history.back();
    }
    return history[moment_ndx];
  }

  bool wait(ulong timestamp)
  {
    ulong         micro_per_sec  = (ulong)1e6;
    timestamp                   /= micro_per_sec;
    unsigned long	diff	         = timestamp - starting_time;
    unsigned long	moment_ndx     = (unsigned long) std::floor(diff / dt_sec);

    if (moment_ndx >= history.size())
      return true;
    else
      return false;
  }

  void write_out(xmlTextWriterPtr writer)
  {
    std::cout << "Writing out sim history" << std::endl;
    for (uint i = 0; i < history.size(); i++)
    {
      xmlTextWriterStartElement(writer, BAD_CAST "moment");
      const moment& this_moment = history[i];
      for (std::map<std::string, _stat>::const_iterator moment_lane = this_moment.begin();
           moment_lane != this_moment.end();
           moment_lane++)
      {
        xmlTextWriterStartElement(writer, BAD_CAST "lane");
        xmlTextWriterWriteAttribute(writer, BAD_CAST "lane_id",	BAD_CAST moment_lane->first.c_str());
        for (uint j = 0; j < moment_lane->second.cells.size(); j++)
        {
          xmlTextWriterStartElement(writer, BAD_CAST "cell");
            xmlTextWriterWriteAttribute(writer, BAD_CAST "end", BAD_CAST "1.0");
            std::ostringstream sout_avg;
            std::ostringstream sout_den;
            sout_avg << moment_lane->second.cells[j].avg_v;
            sout_den << moment_lane->second.cells[j].den;
            xmlTextWriterWriteElement(writer, BAD_CAST "avg", BAD_CAST sout_avg.str().c_str());
            xmlTextWriterWriteElement(writer, BAD_CAST "den", BAD_CAST sout_den.str().c_str());
          xmlTextWriterEndElement(writer);
        }
        xmlTextWriterEndElement(writer);
      }
      xmlTextWriterEndElement(writer);
    }
  }
};
/**************************************************
 * End of structure for simulation history
 **************************************************/


//Macro to call a member function pointer
#define call_member(object, member_ptr) ((object).*(member_ptr))

struct	_lane_stats;

//Typedef for map traversal
typedef std::map<std::string, _lane_stats>::iterator	l_it;
typedef std::pair<std::string, _lane_stats>		element;

//Consts for conversion
const int	min_per_week = 10080;

struct Greenshields
{
  static float f(float vel, float vel_max, float rho_max)
  {
    return rho_max*(1 - (vel/vel_max));
  }
};


struct _point
{
  int	timestamp;
  float vel;
  int	day;
  int	hour;
  float den;

  _point(int in_timestamp, float in_vel):timestamp(in_timestamp), vel(in_vel){}
  _point(){}
};



struct _lane_stats
{
  typedef	void (_lane_stats::*_void_member)();
  typedef	void (_lane_stats::*_writer)(xmlTextWriterPtr);
  typedef int  (_lane_stats::*_int_member)(_time);

public:
  int				count;
  float				avg;
  float				max;
  float				den;
  std::vector<_point>		points;
  l_it				parent;
  std::string			lane_id;
  std::vector<_lane_stats>	children;
  _int_member			get_child_id;
  _void_member			split;
  _writer			write_type;


public:

  _lane_stats()
  {
    split	 = &_lane_stats::temp_split;
    write_type	 = &_lane_stats::write_lane_stats;
    get_child_id = &_lane_stats::get_child_by_day;
    avg		 = 0;
    max		 = 0;
    count	 = 0;
  }

  ~_lane_stats()
  {
  }

  void get_stats(float& _avg, float& _max, float& _den, _time time_t)
  {
    _avg		 = 0;
    _max		 = 0;
    _den                 = 0;
    if (children.size() == 0)
    {
      _avg = avg;
      _max = max;
      _den = den;
    }
    else
    {
      _lane_stats*	child = get_child_for(time_t);
      while(child != NULL)
      {
        _avg = child->avg;
        _max = child->max;
        _den = child->den;

        child = child->get_child_for(time_t);
      }
    }
  }

  _lane_stats* get_child_for(_time time_t)
  {
    //TODO this should be generalized to account for other possible
    //splits at some point.
    _lane_stats*	child  = NULL;
    if (children.size()	      == 0)
    {
      child = NULL;
    }
    else
    {
      int	child_id = call_member(*this, get_child_id)(time_t);
      child		 = &children[child_id];
    }
    return child;
  }

  /*
   *  Polymorphic calls for get_child_id
   */
  int get_child_by_day(_time time_t)
  {
    return time_t.day;
  }

  int get_child_by_hour(_time time_t)
  {
    if (time_t.hour < 6)
      return 0;			//Should not happen...
    else
      return time_t.hour - 6;
  }
  /*
   *  End of calls for get_child_id
   */

  void write_out(xmlTextWriterPtr writer)
  {
    std::cout << "Writing out " << std::endl;
    std::ostringstream	sout_avg;
    std::ostringstream	sout_max;
    int			rc = 0;
    rc			   = xmlTextWriterStartElement(writer, BAD_CAST "lane_stats");
    call_member(*this, write_type)(writer);

    sout_avg << avg;
    xmlTextWriterWriteElement(writer, BAD_CAST "avg", BAD_CAST sout_avg.str().c_str());


    sout_max << max;
    xmlTextWriterWriteElement(writer, BAD_CAST "limit", BAD_CAST sout_max.str().c_str());

    for(unsigned int i = 0; i < children.size(); i++)
    {
      children[i].write_out(writer);
    }

    rc = xmlTextWriterEndElement(writer);
  }

  void insert(_point p)
  {
    float	m_per_s_to_mi_per_h = 2.236936;
    if (p.vel * m_per_s_to_mi_per_h < 100)
    {
      avg = ((avg)*count + p.vel) * (1.0f/(count+1));
      if (p.vel > max)
      {
        max = p.vel;
      }

      den = ((den)*count + p.den) * (1.0f/(count+1));

      count++;
      points.push_back(p);
    }
  }

  void recursive_split()
  {
    call_member(*this, split)();

    for(unsigned int i = 0; i < children.size(); i++)
    {
      call_member(children[i], children[i].split)();
    }
  }

  /*
   *   Various types of splits and accompaunying predicates
   */

  void write_lane_stats(xmlTextWriterPtr writer)
  {
    xmlTextWriterWriteAttribute(writer, BAD_CAST "type_id",
                                BAD_CAST "lane");
    xmlTextWriterWriteAttribute(writer, BAD_CAST "lane_id",
                                BAD_CAST lane_id.c_str());
  }

  void temp_split()
  {
    temp_split_day();
  }

  void write_day_stats(xmlTextWriterPtr writer)
  {
    xmlTextWriterWriteAttribute(writer, BAD_CAST "type_id",
                                BAD_CAST "day");
  }

  void temp_split_day()
  {
    const unsigned int	_WEEK_LEN = 7;
    if (children.size() > 0)
    {
      for (unsigned int i = 0; i < points.size(); i++)
      {
        children[points[i].day].insert(points[i]);
      }
      points.clear();
    }
    else
    {
      const int	_MIN_COUNT = 20;
      int day_count[_WEEK_LEN]   = {0, 0, 0, 0, 0, 0, 0};
      //Bin the points for each day
      std::vector
      <
	    std::vector<_point>
	    >
      day_points;

      //Initialize day_points
      for(unsigned int i = 0; i < _WEEK_LEN; i++)
      {
        day_points.push_back(std::vector<_point>());
      }

      //Count up the points per day to determine whether to split
      for(unsigned int i = 0; i < points.size(); i++)
      {
        _point&	point = points[i];
        day_count[point.day]++;
        day_points[point.day].push_back(points[i]);
      }
      bool	_split = true;
      for (unsigned int i = 0; i < _WEEK_LEN; i++)
      {
        _split = _split and day_count[i] > _MIN_COUNT;
      }

      //If we split, move the points to the children _lane_stats
      if (_split)
      {
        for (unsigned int i = 0; i < _WEEK_LEN; i++)
	      {
          children.push_back(_lane_stats());

          //The child will look for hourly splits
          _lane_stats&	child = children.back();
          child.split	      = &_lane_stats::temp_split_hours;
          child.write_type      = &_lane_stats::write_day_stats;

          //Add the points
          for (unsigned int j = 0; j < day_points[i].size(); j++)
          {
            children.back().insert(day_points[i][j]);
          }

          //Delete the points that were transferred
          day_points[i].clear();
	      }
        points.clear();
      }
  }
  }

  void write_hour_stats(xmlTextWriterPtr writer)
  {
    xmlTextWriterWriteAttribute(writer, BAD_CAST "type_id",
                                BAD_CAST "hour");
  }

void temp_split_hours()
{
  struct _aux
  {
    static int what_hour(_point p)
    {
      //Hours range from 6 to 23
      if (p.hour < 6)
      {
        return 0;		//But just in case...
        std::cerr << "Assumption that point.hours >= 6 is wrong."
                  << std::endl;;
      }
      else
        return p.hour - 6;
    }
  };

  //Hours range from 6 to 23
  const unsigned int	_DAY_HOURS = 18;
  if (children.size() > 0)
  {
    for (unsigned int i = 0; i < points.size(); i++)
    {
      int hour = _aux::what_hour(points[i]);
      children[hour].insert(points[i]);
    }
    points.clear();
  }
  else
  {
    const int	_MIN_COUNT = 20;
    int hour_count[_DAY_HOURS] = {0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0};
    //Bin the points for each day
    std::vector
    <
    std::vector<_point>
    >
    hour_points;

    //Initialize chunk_points
    for(unsigned int i = 0; i < _DAY_HOURS; i++)
    {
      hour_points.push_back(std::vector<_point>());
    }

    //Count up the points per day to determine whether to split
    for(unsigned int i = 0; i < points.size(); i++)
    {
      int hour = _aux::what_hour(points[i]);

      hour_count[hour]++;
      hour_points[hour].push_back(points[i]);
    }
    bool	_split = true;
    for (unsigned int i = 0; i < _DAY_HOURS; i++)
    {
      _split = _split and hour_count[i] > _MIN_COUNT;
    }

    //If we split, move the points to the children _lane_stats
    if (_split)
    {
      for (unsigned int i = 0; i < _DAY_HOURS; i++)
      {
        children.push_back(_lane_stats());

        //The child will look for hourly splits
        _lane_stats&	child = children.back();
        child.split	      = &_lane_stats::temp_split_fine;
        child.write_type      = &_lane_stats::write_hour_stats;

        //Add the points
        for (unsigned int j = 0; j < hour_points[i].size(); j++)
        {
          children.back().insert(hour_points[i][j]);
        }

        //Delete the points that were transferred
        hour_points[i].clear();
      }
      points.clear();
    }
}
  }

  void write_fine_stats(xmlTextWriterPtr writer)
  {
    xmlTextWriterWriteAttribute(writer, BAD_CAST "type_id",
                                BAD_CAST "fine");
  }

  void temp_split_fine()
  {
  }


};


/**************************************************
 * Class to parse stored history data.
 **************************************************/
class _xml_parser : public xmlpp::SaxParser
{
public:
  //member functions
  _xml_parser();
  virtual ~_xml_parser();

  //member data
  _sim_hist* p_history;
  std::string curr_lane;
  bool avg_mode;
  bool den_mode;

protected:
  virtual void on_start_document();
  virtual void on_end_document();
  virtual void on_start_element(const Glib::ustring& name, const AttributeList& properties);
  virtual void on_end_element(const Glib::ustring& name);
  virtual void on_characters(const Glib::ustring& characters);
  virtual void on_comment(const Glib::ustring& text);
  virtual void on_warning(const Glib::ustring& text);
  virtual void on_error(const Glib::ustring& text);
  virtual void on_fatal_error(const Glib::ustring& text);
};

_xml_parser::_xml_parser() : xmlpp::SaxParser() {}
_xml_parser::~_xml_parser(){}


void _xml_parser::on_start_document()
{
  std::cout << "started" << std::endl;
}

void _xml_parser::on_end_document()
{
  std::cout << "ended" << std::endl;
}

void _xml_parser::on_start_element(const Glib::ustring& name,
				   const AttributeList& attributes)
{
  if (name == "moment")
  {
    p_history->history.push_back(_sim_hist::moment());
  }
  if (name == "lane")
  {
    for (xmlpp::SaxParser::AttributeList::const_iterator iter = attributes.begin();
         iter != attributes.end(); iter++)
    {
      if (iter->name == "lane_id")
	    {
        curr_lane = iter->value;
	    }
    }
  }
  if (name == "cell")
  {
    //Add a cell to the current lane of the current moment
    p_history->history.back()[curr_lane].cells.push_back(_cell());
  }
  if (name == "avg")
  {
    //Set flags to read average velocity
    avg_mode = true;
    den_mode = false;
  }
  if (name == "den")
  {
    //Set flags to read average density
    avg_mode = false;
    den_mode = true;
  }
}

void _xml_parser::on_end_element(const Glib::ustring& name)
{
}
void _xml_parser::on_characters(const Glib::ustring& characters)
{
  if (avg_mode)
    {
      float _avg;
      std::istringstream foo(characters);
      foo >> std::dec >> _avg;

      p_history->history.back()[curr_lane].cells.back().avg_v = _avg;

    }
  if (den_mode)
    {
      float _den;
      std::istringstream foo(characters);
      foo >> std::dec >> _den;

      p_history->history.back()[curr_lane].cells.back().den = _den;
    }
}
void _xml_parser::on_comment(const Glib::ustring& text){}
void _xml_parser::on_warning(const Glib::ustring& text){}
void _xml_parser::on_error(const Glib::ustring& text){}
void _xml_parser::on_fatal_error(const Glib::ustring& text){}

/**************************************************
 *   End of XML Parser class
 **************************************************/


#endif
