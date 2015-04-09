#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.H"
#include "arcball.hpp"
#include "hwm_network.hpp"
#include "hwm_draw.hpp"
#include "hybrid-sim.hpp"
#include "big-image-tile.hpp"
#include "sim_history.hpp"
#include <algorithm>
#include <proj_api.h>
#include <ctime>

using namespace std;

//Globalz
big_image *back_image;
vec2i      back_image_dim;
vec2f      back_image_center;
float      back_image_scale;
float      back_image_yscale;
_sim_hist  history;
long       anim_start_time;

int start_hour;
int start_min;
int start_sec;

//forward def
void traffic_cm(float *rgb, const float val);

class I880Parser
{
public:
    std::string filename;

    vec3f center;

    long min_time;
    bool min_time_calc;
    int secs_per_frame;

    I880Parser()
    {
        min_time = std::numeric_limits<long>::max();
        //TODO This is data specific, and must be determined beforehand as loading is in one pass
        min_time = 1202492513;
        //Starting time is 9:41:53 PST
        start_hour = 9;
        start_min = 41;
        start_sec = 53;
        min_time_calc = false;
        std::cout << "min time starts at " << min_time << std::endl;


        //This is an aggregating parameter and determines how course the data collection is.
        secs_per_frame = 1;
    }

    struct _I880point
    {
        float x, y, vel;
        long int time;

      int hour;
      int min;
      int sec;

        _I880point(float _x, float _y, float _vel, long _time)
        {
            x = _x; y = _y; vel = _vel; time = _time;
        }

        void print(ostream& out)
        {
            out << x << ", " << y << ", " << vel << ", " << setprecision(20) << time << std::endl;
        }

    };

    typedef std::map<int, int> _car_count;
    _car_count car_count;

    typedef std::map<int, vector<_I880point> > _I880Data;
    _I880Data data;


    void build_data(float percent, float time_delay)
    {
        //Projections used.
        projPJ pj_merc, pj_latlong;

        if (!(pj_merc = pj_init_plus("+proj=merc +ellps=clrk66 +lat_ts=37.5942 +units=m")) )
            exit(1);
        if (!(pj_latlong = pj_init_plus("+proj=latlong +ellps=clrk66")) )
            exit(1);

        double deg_to_rad = M_PI / 180.0;

        //Used when only a certain percentage of the cars will be considered.
        int mod_num = (int) (1.0 / ( percent / 100.0 ) );

        //Open the file named in the class variable 'filename'
        std::ifstream fin(filename.c_str());
        if (!fin) { std::cout << "Error opening file." << std::endl;return;}

        double fudge_x     = 0;
        double fudge_y     = 0;
        int    first_run   = 1;
        int    count       = 0;
        float  time_passed = 0;
        int    frame       = 0;

        //Not currently used
        int _first_indx = std::numeric_limits<int>::max();
        int _last_indx  = std::numeric_limits<int>::min();

        //Parse the file
        ///Throw out first line
        std::string null;
        getline(fin, null, '\n');

        //Read a time, lat, lon, mile, and vel repeatedly.
        std::string s_u_time;
        while(getline(fin, s_u_time, ','))
        {
            std::string s_lat;
            std::string s_lon;
            std::string s_postmile;
            std::string s_vel;
            getline(fin, s_lat, ',');
            getline(fin, s_lon, ',');
            getline(fin, s_postmile, ',');
            getline(fin, s_vel, ',');
            getline(fin, null, '\n');

            long unix_time;
            {
                stringstream ssline(s_u_time);
                ssline >> unix_time;

                int rel_time = unix_time;
                //Subtract the PRECOMPUTED min_time
                if (!min_time_calc)
                    rel_time -= min_time;

                //Calculate which frame this is put in
                frame = rel_time / secs_per_frame;
            }

            double lat;
            {
                stringstream ssline(s_lat);
                ssline >> lat;
            }


            double lon;
            {
                stringstream ssline(s_lon);
                ssline >> lon;
            }


            double postmile;
            {
                stringstream ssline(s_postmile);
                ssline >> postmile;
            }

            float vel;
            {
                stringstream ssline(s_vel);
                ssline >> vel;
            }

            //Todo this may not be appropriate anymore.
            if (first_run == 1)
            {
              first_run = 0;
              time_passed = (rand() / (float)RAND_MAX) * time_delay;
              count++;
            }

            //Used to get the minimum time used in the data.
            if (min_time_calc)
            {
                if (unix_time < min_time)
                {
                    min_time = unix_time;
                }
            }

            time_passed += 0.1;

            //This is used to artificially shrink the number
            // or probe vehicles being used
            if (count % mod_num == 0)
            {
                //This is used to artificially delay the
                // the signals sent to match a time delay
                if (time_passed >= time_delay)
                {
                    time_passed = 0;

                    //Project the lat-lon to Cartesian space.
                    /// Projection is done here for efficiency.
                    lat *= deg_to_rad;
                    lon *= deg_to_rad;
                    pj_transform(pj_latlong, pj_merc, 1, 1, &lon, &lat, NULL);

                    //Store the data point.
                    data[frame].push_back(_I880point(lon - center[0] - fudge_x,
                                                     lat - center[1] - fudge_y,
                                                     vel,
                                                     unix_time));
                    data[frame].back().hour =  ((unix_time - min_time) / (60*60));
                    data[frame].back().min = ((unix_time - min_time) / (60) - ((unix_time - 1202492513) / (60*60))*60);
                    data[frame].back().sec =  (unix_time - min_time) -  (data[frame].back().min * 60) - (data[frame].back().hour) * 60 * 60;


                    //                    std::cout << "time " << unix_time << " is " <<  data[frame].back().hour << ":" <<  data[frame].back().min  << ":" << data[frame].back().sec << std::endl;
                    //                    std::cout << "min now " << min_time << std::endl;
                }
            }
        }

        //This was used to create density-velocity plots for I80 data.
        /*
        typedef float den;
        typedef float vel;

        vector<vector<pair<den, vel> > > den_vel_pairs;
        for (int i = 0; i <= 6; i++)
        {
            den_vel_pairs.push_back(vector<pair<den, vel> >());
        }

        vector<ofstream*> fouts;
        for (int i = 0; i <= 6; i++)
        {
            stringstream sin;
            sin << "lane_data_" << i << ".csv";
            ofstream* fout = new ofstream;
            fout->open(sin.str().c_str());
            fouts.push_back(fout);
        }

        for (int indx = _first_indx; indx <= _last_indx; indx++)
        {
            int           frame_num = indx;
            vector<int>   counts;
            vector<float> vel_sum;
            vector<vector<float > > vels;
            vector<float> spacing_sum;
            for (int i = 0; i <= 6; i++)
            {
                counts.push_back(0);
                vel_sum.push_back(0);
                vels.push_back(vector<float>());
                spacing_sum.push_back(0);
            }
            for (int i = 0; i < data[frame_num].size(); i++)
            {
                counts     [data[frame_num][i].lane - 1]++;
                vel_sum    [data[frame_num][i].lane - 1] += data[frame_num][i].vel;
                vels       [data[frame_num][i].lane - 1].push_back(data[frame_num][i].vel);
                spacing_sum[data[frame_num][i].lane - 1] += data[frame_num][i].spacing;
            }

            for(int i = 0; i <= 6; i++)
            {
                sort(vels[i].begin(), vels[i].end());
            }

            for (int i = 0; i <= 6; i++)
            {
                if ((vels[i].size() > 0) and (counts[i] > 2))
                {
                    *fouts[i] << vel_sum[i] / (float) counts[i] << " " << vels[i][vels[i].size() / 2] << " " << counts[i] / (float) spacing_sum[i] << std::endl;
                    den_vel_pairs[i].push_back(pair<den, vel>(counts[i] / (float) spacing_sum[i],
                                                         vels[i][vels[i].size() / 2]));
                }

//                 std::cout << "cars in lane " << i + 1 << " at timestep " << frame_num << " "  << counts[i] << std::endl;
//                 std::cout << "   and avg vel " << vel_sum[i] / (float) counts[i] << std::endl;
//                 std::cout << "   and spacing " << spacing_sum[i] << std::endl;
//                 std::cout << "   and med. v. " << vels[i][vels[i].size() / 2] << std::endl;
            }
        }
        */
    }

    void print_sizes()
    {
        ofstream fout("data_hist.csv");
        for (_I880Data::iterator it = data.begin(); it != data.end(); it++)
        {
            fout << it->first << ", " << it->second.size() << std::endl;
        }
    }

    void print_in_order(ostream& out)
    {
        for (_I880Data::iterator it = data.begin(); it != data.end(); it++)
        {
            for (int i = 0; i < it->second.size(); i++)
            {
                out << it->first << ", ";
                it->second[i].print(out);
            }
        }
    }

    // void plot_lane_point(int l, float x)
    // {
    //     float meter_to_feet = 3.2808399;
    //     x *= meter_to_feet;
    //     float delta_x = 20 * meter_to_feet;
    //     int count = 0;

    //     ofstream fout;
    //     fout.open("v_x.csv");

    //     for (_I880Data::iterator it = data.begin(); it != data.end(); it++)
    //     {
    //         for (int i = 0; i < it->second.size(); i++)
    //         {
    //             if (it->second[i].lane == l)
    //             {
    //                 if (abs(it->second[i].local_y - x) < delta_x)
    //                 {
    //                     fout << it->first * 0.1 << ", ";
    //                     fout << it->second[i].vel << std::endl;
    //                     count++;
    //                 }
    //             }
    //         }
    //     }
    //     std::cout << "Found " << count << std::endl;
    // }

    void draw_points()
    {
      long curr_time = time(0);
      int sec = curr_time - anim_start_time;
      std::cout << "time passed " << curr_time - anim_start_time << std::endl;

        glBegin(GL_POINTS);

        int min = std::numeric_limits<int>::max();
        int max = std::numeric_limits<int>::min();
        for(_I880Data::iterator _it = data.begin();
            _it != data.end();
            _it++)
        {

            for (int i = 0; i < _it->second.size(); i++)
            {
              //              if  ( _it->second[i].hour == 6 and _it->second[i].min > (sec - 1) and _it->second[i].min < (sec + 1))
              if  ( _it->second[i].hour == 0 and _it->second[i].min > 50 and _it->second[i].min < 60)
              {
                if (_it->first < min)
                  min = _it->first;
                if (_it->first > max)
                  max = _it->first;

                float rgb[3];
                traffic_cm(rgb, _it->second[i].vel);
                glColor3f(rgb[0], rgb[1], rgb[2]);
                glVertex2f(_it->second[i].x, _it->second[i].y);
              }
            }
        }
        std::cout << "min and max frames were " << min << " and " << max << std::endl;

        glEnd();

    }
};

static GLint biggest_texture()
{
    GLint biggest_width = 64;
    GLint width = 1;
    while ( width ) /* use a better condition to prevent possible endless loop */
    {
        glTexImage2D(GL_PROXY_TEXTURE_2D,
                     0,                /* mip map level */
                     GL_RGBA,          /* internal format */
                     biggest_width,     /* width of image */
                     biggest_width,    /* height of image */
                     0,                /* texture border */
                     GL_RGBA,          /* pixel data format, */
                     GL_UNSIGNED_BYTE, /* pixel data type */
                     NULL              /* null pointer because this a proxy texture */
                     );

        /* the queried width will NOT be 0, if the texture format is supported */
        glGetTexLevelParameteriv(GL_PROXY_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);

        ++biggest_width;
    }
    return biggest_width;
}

I880Parser parser;

static inline void blackbody(float *rgb, const float val)
{
   if(val <= 0.0) // clamp low to black
       rgb[0] = rgb[1] = rgb[2] = 0.0f;
   else if(val >= 1.0f) // and high to white
       rgb[0] = rgb[1] = rgb[2] = 1.0f;
   else if(val < 1.0f/3.0f) // go to [1, 0, 0] over [0, 1/3)
   {
       rgb[0] = val*3.0f;
       rgb[1] = 0.0f;
       rgb[2] = 0.0f;
   }
   else if(val < 2.0f/3.0f)  // go to [1, 1, 0] over [1/3, 2/3)
   {
       rgb[0] = 1.0f;
       rgb[1] = (val-1.0f/3.0f)*3.0f;
       rgb[2] = 0.0f;
   }
   else // go to [1, 1, 1] over [2/3, 1.0)
   {
       rgb[0] = 1.0f;
       rgb[1] = 1.0f;
       rgb[2] = (val-2.0f/3.0f)*3.0f;
   }
   return;
}

void traffic_cm(float *rgb, const float val)
{
  if (val < 0)
  {
    rgb[0] = rgb[1] = rgb[2] = 0.8f;
    rgb[3] = 0.0f;
  }
  else if(val <= 11.1760) // clamp low to red
  {
    rgb[0] = 1;
    rgb[1] = rgb[2] = 0.0f;
  }
  else if(val >= 22.3520f) // and high to green
  {
    rgb[1] = 1.0f;
    rgb[0] = rgb[2] = 0.0f;
  }
  else
  {
    rgb[0] = 1.0f;
    rgb[1] = 1.0f;
    rgb[2] = 0.0f;
  }

  return;
}



static const float CAR_LENGTH = 4.5f;
//* This is the position of the car's axle from the FRONT bumper of the car
static const float CAR_REAR_AXLE = 3.5f;

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          zoom(12.941),
                                                          car_pos(0.0f),
                                                          glew_state(GLEW_OK+1),
                                                          light_position(50.0, 100.0, 50.0, 1.0),
                                                          tex_(0),
                                                          sim(0),
                                                          drawfield(RHO)
    {
        lastmouse[0] = 0.0f;
        lastmouse[1] = 0.0f;

        this->resizable(this);
    }

    void setup_light()
    {
        static const GLfloat amb_light_rgba[] = { 0.1, 0.1, 0.1, 1.0 };
        static const GLfloat diff_light_rgba[] = { 0.1, 0.1, 0.1, 1.0 };
        static const GLfloat spec_light_rgba[] = { 0.1, 0.1, 0.1, 1.0 };
        static const GLfloat spec_material[] = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat material[] = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat shininess = 100.0;

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_COLOR_MATERIAL);
        glPushMatrix();
        glLoadIdentity();
        glLightfv(GL_LIGHT0, GL_POSITION, light_position.data());
        glPopMatrix();
        glLightfv(GL_LIGHT0, GL_AMBIENT, amb_light_rgba );
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diff_light_rgba );
        glLightfv(GL_LIGHT0, GL_SPECULAR, spec_light_rgba );
        glMaterialfv( GL_FRONT, GL_AMBIENT, material );
        glMaterialfv( GL_FRONT, GL_DIFFUSE, material );
        glMaterialfv( GL_FRONT, GL_SPECULAR, spec_material );
        glMaterialfv( GL_FRONT, GL_SHININESS, &shininess);
    }

    void init_glew()
    {
        glew_state = glewInit();
        if (GLEW_OK != glew_state)
        {
            /* Problem: glewInit failed, something is seriously wrong. */
            std::cerr << "Error: " << glewGetErrorString(glew_state)  << std::endl;
        }
        std::cerr << "Status: Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;
    }

    void init_textures()
    {
        GLint biggest_width = biggest_texture();
        std::cout << "Largest texture I support: " << biggest_width << std::endl;

        if(back_image && back_image->tiles.empty())
        {
            back_image->make_tiles(biggest_width/2, false);
        }

        if(!glIsTexture(tex_))
        {
            glGenTextures(1, &tex_);
            glEnable(GL_TEXTURE_2D);
            glBindTexture (GL_TEXTURE_2D, tex_);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            glDisable(GL_TEXTURE_2D);
        }
    }

    void draw()
    {
        if (!valid())
        {
            glViewport(0, 0, w(), h());
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(45.0f, (GLfloat)w()/(GLfloat)h(), 200.0f, 16000.0f);
            //            glTranslatef(1000.0f, 0.0f, -6000);

            glMatrixMode(GL_MODELVIEW);
            glClearColor(0.0, 0.0, 0.0, 0.0);

            glEnable(GL_DEPTH_TEST);

            glHint(GL_LINE_SMOOTH_HINT,    GL_NICEST);
            glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

            glEnable(GL_LINE_SMOOTH);

            if(GLEW_OK != glew_state)
                init_glew();

            if(!car_drawer.initialized())
                car_drawer.initialize(0.6*sim->hnet->lane_width,
                                      CAR_LENGTH,
                                      1.5f,
                                      CAR_REAR_AXLE);

            if(!network_drawer.initialized())
                network_drawer.initialize(sim->hnet, 0.01f);

            glDisable(GL_LIGHTING);
            init_textures();
            glEnable(GL_TEXTURE_2D);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisable(GL_LIGHTING);
            glDepthFunc(GL_LEQUAL);


            setup_light();

            if(sim)
            {
                bb[0] = vec3f(FLT_MAX);
                bb[1] = vec3f(-FLT_MAX);
                sim->hnet->bounding_box(bb[0], bb[1]);
            }

            anim_start_time = time(0);
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();

        glTranslatef(0.0f, 0.0f, -std::pow(2.0f, zoom));

        nav.get_rotation();
        glMultMatrixd(nav.world_mat());

        glLightfv(GL_LIGHT0, GL_POSITION, light_position.data());

        if(sim)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDisable(GL_LIGHTING);

            glColor3f(1.0, 1.0, 1.0);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glEnable(GL_LIGHTING);

            glEnable(GL_TEXTURE_2D);

            glColor4f(1.0, 1.0, 1.0, 1.0);
            if(back_image && !back_image->tiles.empty())
            {
                glPushMatrix();
                glTranslatef(-back_image_center[0], -back_image_center[1], -0.5);
                glScalef    (back_image_scale, back_image_yscale*back_image_scale,   1);

                vec2i dim(back_image->dim());
                glTranslatef(-dim[0]/2, dim[1]/2, 0);
                back_image->draw();
                glPopMatrix();
            }

            glDisable(GL_LIGHTING);
            glEnable(GL_TEXTURE_2D);
            std::vector<vec4f> colors;
            BOOST_FOREACH(hybrid::lane &l, sim->lanes)
            {
              if (l.parent->length() < 1000)
                continue;

                if(!l.parent->active)
                    continue;

                glBindTexture (GL_TEXTURE_2D, tex_);

                long curr_time = time(0);
                int sec = curr_time - anim_start_time;

                uint current_moment = std::min(sec, (int)history.history.size());
                _sim_hist::moment current_state = history.history[current_moment];
                _stat lane_state = current_state[l.parent->id];

                bool should_draw = true;

                colors.resize(lane_state.cells.size());
                for(size_t i = 0; i < lane_state.cells.size(); ++i)
                {
                  float avg_v = lane_state.cells[i].avg_v;



                  colors[i][3] = 1.0f;
                  traffic_cm(colors[i].data(), avg_v);

                  if (colors[i][3] <= 0)
                  {
                    should_draw = false;
                  }

                  //TMP
                  // if (lane_state.cells[i].den > 0)
                  // {
                  //   colors[i][0] = 0; colors[i][1] = 0; colors[i][2] = 0;
                  // }
                  // else
                  // {
                  //   colors[i][0] = 1; colors[i][1] = 1; colors[i][2] = 1;
                  // }
                }

                if (should_draw)
                {
                  glTexImage2D (GL_TEXTURE_2D,
                                0,
                                GL_RGBA,
                                lane_state.cells.size(),
                                1,
                                0,
                                GL_RGBA,
                                GL_FLOAT,
                                colors[0].data());

                  std::cout << "Frame " << current_moment << std::endl;
                  network_drawer.draw_lane_solid(l.parent->id);
                }
            }
            glDisable(GL_TEXTURE_2D);

            //     BOOST_FOREACH(hybrid::lane &l, sim->lanes)
            //     {
            //         if(!l.parent->active)
            //             continue;

            //         glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            //         glDisable(GL_LIGHTING);
            //         network_drawer.draw_lane_solid(l.parent->id);

            //         glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            //         glEnable(GL_LIGHTING);

            //         BOOST_FOREACH(const hybrid::car &c, l.current_cars())
            //         {
            //             assert(c.position >= 0);
            //             assert(c.position < 1.0);
            //             mat4x4f trans(l.parent->point_frame(c.position));
            //             mat4x4f ttrans(tvmet::trans(trans));

            //             glPushMatrix();
            //             glMultMatrixf(ttrans.data());
            //             car_drawer.draw();
            //             glPopMatrix();
            //         }
            //     }

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDisable(GL_LIGHTING);
            network_drawer.draw_intersections_wire();

            glEnable(GL_BLEND);
            glDisable(GL_LIGHTING);
        }

        //        parser.draw_points();

        redraw();
        glFlush();
        glFinish();
    }

    int handle(int event)
    {
        switch(event)
        {
        case FL_PUSH:
            {
                int x = Fl::event_x();
                int y = Fl::event_y();
                float fx =   2.0f*x/(w()-1) - 1.0f;
                float fy = -(2.0f*y/(h()-1) - 1.0f);
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    nav.get_click(fx, fy);
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                }
                else if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                }
                lastmouse[0] = fx;
                lastmouse[1] = fy;
                redraw();
            }
            take_focus();
            return 1;
        case FL_DRAG:
            {
                int x = Fl::event_x();
                int y = Fl::event_y();
                float fx =  2.0f*x/(w()-1)-1.0f;
                float fy = -(2.0f*y/(h()-1)-1.0f);
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    float scale = std::pow(2.0f, zoom-1.0f);

                    float update[3] = {
                        (fx-lastmouse[0])*scale,
                        (fy-lastmouse[1])*scale,
                        0.0f
                    };

                    back_image_center[0] -= update[0];
                    back_image_center[1] -= update[1];

                    std::cout << back_image_center[0] << " " << back_image_center[1] << std::endl;
                    redraw();

                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                    float scale = std::pow(2.0f, zoom-1.0f);

                    double update[3] = {
                        (fx-lastmouse[0])*scale,
                        (fy-lastmouse[1])*scale,
                        0.0f
                    };

                    nav.translate(update);
                }
                else if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                    float scale = std::pow(1.5f, zoom-1.0f);
                    zoom += scale*(fy-lastmouse[1]);
                    if(zoom > 17.0f)
                        zoom = 17.0f;
                    else if(zoom < FLT_MIN)
                        zoom = FLT_MIN;
                }
                lastmouse[0] = fx;
                lastmouse[1] = fy;
                redraw();
            }
            take_focus();
            return 1;
        case FL_KEYBOARD:
            switch(Fl::event_key())
            {
            case 'n':
                if(sim)
                {
                    sim->advance_intersections(0.5);
                }
                break;
            case ' ':
                if(sim)
                {
                    sim->hybrid_step();
                }
                break;
            case 'p':
                drawfield = RHO;
                break;
            case 'u':
                drawfield = U;
                break;
            case 't':
                car_pos += 0.02f;
                if(car_pos > 1.0f)
                    car_pos = 1.0f;
                break;
            case 'g':
                car_pos -= 0.02f;
                if(car_pos < 0.0f)
                    car_pos = 0.0f;
                break;
            case 'c':
                if(sim)
                {
                    sim->convert_cars(hybrid::MACRO);
                }
                break;
            case 'm':
                if(sim)
                {
                    BOOST_FOREACH(hybrid::lane &l, sim->lanes)
                    {
                        if(l.sim_type != hybrid::MACRO)
                            continue;

                        l.macro_instantiate(*sim);

                        float pos;
                        if(l.macro_find_first(pos, *sim))
                            std::cout << "Lane: " << l.parent->id << "first at " << pos << std::endl;
                        else
                            std::cout << "Lane: " << l.parent->id << "empty. (no first)" << std::endl;

                        if(l.macro_find_last(pos, *sim))
                            std::cout << "Lane: " << l.parent->id << "last at " << pos << std::endl;
                        else
                            std::cout << "Lane: " << l.parent->id << "empty. (no last)" << std::endl;
                    }
                }
                break;
            default:
                break;
            }
            redraw();
            return 1;
        case FL_MOUSEWHEEL:
            {

                const vec2i xy(Fl::event_x(),
                               Fl::event_y());
                const vec2i dxy(Fl::event_dx(),
                                Fl::event_dy());
                const float fy = copysign(0.5, dxy[1]);

                back_image_scale  *= std::pow(2.0f, 0.01f*fy);
                std::cout << "scale " << back_image_scale << std::endl;
                redraw();
            }
            take_focus();
            return 1;
        default:
            // pass other events to the base class...
            return Fl_Gl_Window::handle(event);
        }
    }

    arcball nav;
    float zoom;
    float lastmouse[2];

    big_image *back_image;
    vec2i      back_image_dim;
    vec2f      back_image_center;
    float      back_image_scale;
    float      back_image_yscale;


    float             car_pos;
    hwm::car_draw     car_drawer;
    hwm::network_draw network_drawer;

    vec3f bb[2];

    GLuint            glew_state;
    vec4f             light_position;
    GLuint            tex_;
    hybrid::simulator *sim;
    typedef enum {RHO, U} draw_type;
    draw_type         drawfield;
};

fltkview* _win;
float rate = 0.5;
void frame_callback(void*)
{
  _win->draw();
  Fl::repeat_timeout(rate, frame_callback);

}

int main(int argc, char *argv[])
{
    std::cerr << libhybrid_package_string() << std::endl;
    if(argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <network file> <image file> " << std::endl;
        return 1;
    }

    bool start_files = false;
    vector<string> files;
    for (int i = 0; i < argc; i++)
    {
        if (start_files)
            files.push_back(argv[i]);
        if (strcmp(argv[i],"-f") == 0)
            start_files = true;
    }

    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0, 1.0, 1.0f)));
    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();

    std::cerr << "HWM net loaded successfully" << std::endl;

    parser.center[0] = net.center_point[0];
    parser.center[1] = net.center_point[1];
    std::cout << "Files found " << files.size() << std::endl;
    for (int i = 0; i < files.size(); i++)
    {
        parser.filename = files[i];
        parser.build_data(100, 0);
    }
    parser.print_sizes();
    std::cout << parser.min_time << " is min time" << std::endl;

    ofstream fouty;
    fouty.open("berk_gps.txt");
    parser.print_in_order(fouty);

    try
    {
        net.check();
        std::cerr << "HWM net checks out" << std::endl;
    }
    catch(std::runtime_error &e)
    {
        std::cerr << "HWM net doesn't check out: " << e.what() << std::endl;
        exit(1);
    }

    //Load history of simulation
    std::string	file  = "sim_history.xml";
    _xml_parser	hist_parser;
    hist_parser.p_history = &history;
    hist_parser.parse_file(file);

    int max = 0;
    for (int i = 0 ; i < history.history.size(); i++)
    {
      for (_sim_hist::moment::iterator foo = history.history[i].begin();
           foo != history.history[i].end();
           foo++)
      {
        if (foo->second.cells.size() > max)
        {
          max = foo->second.cells.size();
        }
      }
    }
    std::cout << "max cells is " << max << std::endl;

    //write out for sanity check
    xmlTextWriterPtr writer = xmlNewTextWriterFilename("sim_history_chk.xml", 0);
    xmlTextWriterStartDocument(writer, NULL, "ISO-8859-1", NULL);
    xmlTextWriterStartElement(writer, BAD_CAST "db");
    history.write_out(writer);
    xmlTextWriterEndElement(writer);
    xmlTextWriterEndDocument(writer);
    xmlFreeTextWriter(writer);

    hybrid::simulator s(&net,
                        4.5,
                        1.0);
    s.micro_initialize(0.73,
                       1.67,
                       33,
                       4);
    s.macro_initialize(0.5, 2.1*4.5, 0.0f);

    fltkview mv(0, 0, 500, 500, "fltk View");
    _win = &mv;
    mv.sim = &s;

    mv.back_image = new big_image(argv[2]);
    mv.back_image_center = vec2f(1814.01f, -428.356f);
    mv.back_image_scale =  3.77295;
    mv.back_image_yscale = 1.0;

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    Fl::add_timeout(rate, frame_callback);
    return Fl::run();
}
