#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.h"
#include "arcball.hpp"
#include "libroad/hwm_network.hpp"
#include "libroad/hwm_draw.hpp"
#include "libhybrid/hybrid-sim.hpp"
#include "big-image-tile.hpp"
#include <algorithm>

using namespace std;



//Globalz
big_image *back_image;
vec2i      back_image_dim;
vec2f      back_image_center;
float      back_image_scale;
float      back_image_yscale;


class I80Parser
{
public:
    std::string filename;

    vec3f center;

    struct _I80point
    {
        float x, y, vel, local_y;
        long int time;
        int lane;
        float spacing;

        _I80point(float _x, float _y, float _vel, float _local_y, long int _time, int _lane, float _spacing)
        {
            x = _x; y = _y; vel = _vel; local_y = _local_y; time = _time; lane = _lane; spacing = _spacing;
        }

        void print(ostream& out)
        {
            out << x << ", " << y << ", " << vel << ", " << local_y << ", " << setprecision(20) << time << std::endl;
        }

    };

    typedef std::map<int, int> _car_count;
    _car_count car_count;

    typedef std::map<int, vector<_I80point> > _I80Data;
    _I80Data data;



    void build_data(float percent, float time_delay)
    {
        float max_vel = 0;
        float max_local_y = 0;
        int mod_num = (int) (1.0 / ( percent / 100.0 ) );

        float foot_to_meter = 0.3048; //1200.0/3937.0; //s. feet/meters

        std::ifstream fin(filename.c_str());
        if (!fin) { std::cout << "Error opening file." << std::endl;return;}

        double fudge_x = 0;
        double fudge_y = 215;

        int last_car_id = -1;
        int count = 0;

        float time_passed = 0;

        int _first_indx = std::numeric_limits<int>::max();
        int _last_indx = std::numeric_limits<int>::min();

        std::string line;
        while(getline(fin, line, '\n'))
        {
            stringstream ssline(line);

            int car_id;
            ssline >> car_id;

            if (last_car_id == -1 or last_car_id != car_id)
            {
              last_car_id = car_id;
              time_passed = (rand() / (float)RAND_MAX) * time_delay;
              count++;
            }

            int frame;
            ssline >> frame;
            if (frame < _first_indx)
                _first_indx = frame;
            if (frame > _last_indx)
                _last_indx = frame;

            time_passed += 0.1;

            int total_frames;
            ssline >> total_frames;

            long epoch_time;
            ssline >> epoch_time;

            float local_x;
            ssline >> local_x;
            local_x *= foot_to_meter;

            float local_y;
            ssline >> local_y;
            local_y *= foot_to_meter;
            if (local_y > max_local_y)
                max_local_y = local_y;

            double global_x;
            ssline >> global_x;
            global_x *= foot_to_meter;

            double global_y;
            ssline >> global_y;
            global_y *= foot_to_meter;

            float length_ft;
            ssline >> length_ft;

            float width_ft;
            ssline >> width_ft;

            int veh_class;
            ssline >> veh_class;

            float velocity;
            ssline >> velocity;
            velocity *= foot_to_meter;

            if (velocity > max_vel)
            {
                max_vel = velocity;
            }

            float accel;
            ssline >> accel;

            int lane_num;
            ssline >> lane_num;

            int prec_veh;
            ssline >> prec_veh;

            int fol_veh;
            ssline >> fol_veh;

            float spacing;
            ssline >> spacing;
            spacing *= foot_to_meter;

            float headway;
            ssline >> headway;


            if (veh_class != 1)
            {
                if (count % mod_num == 0)
                {
                    if (time_passed >= time_delay)
                    {
                        time_passed = 0;
                        data[frame].push_back(_I80point(global_x - center[0] - fudge_x,
                                                        global_y - center[1] - fudge_y,
                                                        velocity,
                                                        local_y,
                                                        epoch_time,
                                                        lane_num,
                                                        spacing));
                        car_count[count] = 1;
                    }
                }
            }
        }


        std::cout << "Cars found : " << car_count.size() << " or " << (car_count.size() / (float)count) * 100 << "%"<< std::endl;
        std::cout << "max vel: " << max_vel << std::endl;
        std::cout << "max localy: " << max_local_y << std::endl;

        //Data outputting
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
    }

    void print_in_order(ostream& out)
    {
        for (_I80Data::iterator it = data.begin(); it != data.end(); it++)
        {
            for (int i = 0; i < it->second.size(); i++)
            {
                out << it->first << ", ";
                it->second[i].print(out);
            }
        }

    }

    void plot_lane_point(int l, float x)
    {
        float meter_to_feet = 3.2808399;
        x *= meter_to_feet;
        float delta_x = 20 * meter_to_feet;
        int count = 0;

        ofstream fout;
        fout.open("v_x.csv");

        for (_I80Data::iterator it = data.begin(); it != data.end(); it++)
        {
            for (int i = 0; i < it->second.size(); i++)
            {
                if (it->second[i].lane == l)
                {
                    if (abs(it->second[i].local_y - x) < delta_x)
                    {
                        fout << it->first * 0.1 << ", ";
                        fout << it->second[i].vel << std::endl;
                        count++;
                    }
                }
            }
        }
        std::cout << "Found " << count << std::endl;
    }

    void draw_points()
    {
        glBegin(GL_POINTS);

        for(_I80Data::iterator _it = data.begin();
            _it != data.end();
            _it++)
        {
            //            std::cout << _it->first << " has " << _it->second.size() << endl;
            for (int i = 0; i < _it->second.size(); i++)
            {
                glColor3f(0.0, _it->second[i].vel, 0.0);
                glVertex2f(_it->second[i].x, _it->second[i].y);
            }
        }

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


I80Parser parser;


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

static const float CAR_LENGTH = 4.5f;
//* This is the position of the car's axle from the FRONT bumper of the car
static const float CAR_REAR_AXLE = 3.5f;

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          zoom(2.0),
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
        static const GLfloat diff_light_rgba[] = { 0.7, 0.7, 0.7, 1.0 };
        static const GLfloat spec_light_rgba[] = { 1.0, 1.0, 1.0, 1.0 };
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

//         if(!glIsTexture(tex_))
//         {
//             glGenTextures(1, &tex_);
//             glEnable(GL_TEXTURE_2D);
//             glBindTexture (GL_TEXTURE_2D, tex_);
//             glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
//             glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
//             glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
//             glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
//             glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
//             glDisable(GL_TEXTURE_2D);
//         }
    }

    void draw()
    {
        if (!valid())
        {
            glViewport(0, 0, w(), h());
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(45.0f, (GLfloat)w()/(GLfloat)h(), 10.0f, 5000.0f);

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

            init_textures();
            //             glEnable(GL_TEXTURE_2D);
            //             glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
            //             glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            //             glDisable(GL_LIGHTING);
            //             glDepthFunc(GL_LEQUAL);


            setup_light();

            if(sim)
            {
                bb[0] = vec3f(FLT_MAX);
                bb[1] = vec3f(-FLT_MAX);
                sim->hnet->bounding_box(bb[0], bb[1]);
            }
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


            std::vector<vec4f> colors;
            BOOST_FOREACH(hybrid::lane &l, sim->lanes)
            {
                if(!l.parent->active)
                    continue;

                glBindTexture (GL_TEXTURE_2D, tex_);

                colors.resize(l.N);
                for(size_t i = 0; i < l.N; ++i)
                {
                    float val;
                    if(drawfield == RHO)
                        val = l.q[i].rho();
                    else
                        val = arz<float>::eq::u(l.q[i].rho(),
                                                l.q[i].y(),
                                                l.parent->speedlimit,
                                                sim->gamma)/l.parent->speedlimit;

                    blackbody(colors[i].data(), val);
                    colors[i][3] = 1.0f;
                }

                glTexImage2D (GL_TEXTURE_2D,
                              0,
                              GL_RGBA,
                              l.N,
                              1,
                              0,
                              GL_RGBA,
                              GL_FLOAT,
                              colors[0].data());


                glColor3f(0.0, 0.0, 1.0);
                network_drawer.draw_lane_wire(l.parent->id);
            }
            glDisable(GL_TEXTURE_2D);

            BOOST_FOREACH(hybrid::lane &l, sim->lanes)
            {
                if(!l.parent->active)
                    continue;

                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glDisable(GL_LIGHTING);
                network_drawer.draw_lane_wire(l.parent->id);

                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                glEnable(GL_LIGHTING);


                BOOST_FOREACH(const hybrid::car &c, l.current_cars())
                {
                    assert(c.position >= 0);
                    assert(c.position < 1.0);
                    mat4x4f trans(l.parent->point_frame(c.position));
                    mat4x4f ttrans(tvmet::trans(trans));

                    glPushMatrix();
                    glMultMatrixf(ttrans.data());
                    car_drawer.draw();
                    glPopMatrix();
                }
            }

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDisable(GL_LIGHTING);
            glColor3f(0.0, 0.0, 1.0);
            network_drawer.draw_intersections_wire();


            glEnable(GL_BLEND);
            glDisable(GL_LIGHTING);
        }


        parser.draw_points();

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

                back_image_scale  *= std::pow(2.0f, 0.1f*fy);
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

int main(int argc, char *argv[])
{
    std::cerr << libhybrid_package_string() << std::endl;
    if(argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <network file> <image file> " << std::endl;
        return 1;
    }

    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0, 1.0, 1.0f)));
    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();

    std::cerr << "HWM net loaded successfully" << std::endl;

    parser.filename = "trajectories-0400-0415.txt";
    double survey_foot_to_foot = (1.0 / 0.99999803149994);
    parser.center[0] = net.center_point[0] * survey_foot_to_foot;
    parser.center[1] = net.center_point[1] * survey_foot_to_foot;
    parser.build_data(100, 0);
    ofstream fout;
    fout.open("pseudo_gps.txt");
    parser.print_in_order(fout);
    //parser.plot_lane_point(1, 800 * 0.3048);

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

    hybrid::simulator s(&net,
                        4.5,
                        1.0);
    s.micro_initialize(0.73,
                       1.67,
                       33,
                       4);
    s.macro_initialize(0.5, 2.1*4.5, 0.0f);

//     {
//         hybrid::lane &l = s.get_lane_by_name("lane0b");
//         l.sim_type = hybrid::MICRO;

//         const int cars_per_lane = 0;
//         double    p             = -s.rear_bumper_offset()*l.inv_length;

//         for (int i = 0; i < cars_per_lane; i++)
//         {
//             //TODO Just creating some cars here...
//             l.current_cars().push_back(s.make_car(p, 10, 0));

//             //Cars need a minimal distance spacing
//             p += (25.0 * l.inv_length);
//             if(p + s.front_bumper_offset()*l.inv_length >= 1.0)
//                 break;
//         }
//     }

//     {
//         hybrid::lane &l = s.get_lane_by_name("lane0a");
//         l.sim_type = hybrid::MACRO;

//         const int cars_per_lane = 8;
//         double    p             = -s.rear_bumper_offset()*l.inv_length;

//         for (int i = 0; i < cars_per_lane; i++)
//         {
//             //TODO Just creating some cars here...
//             l.current_cars().push_back(s.make_car(p, 0, 0));

//             //Cars need a minimal distance spacing
//             p += (6.0 * l.inv_length);
//             if(p + s.front_bumper_offset()*l.inv_length >= 1.0)
//                 break;
//         }
//     }

//    s.settle(0.033);

    s.convert_cars(hybrid::MACRO);

    BOOST_FOREACH(hybrid::lane &l, s.lanes)
    {
        if(l.is_macro())
            l.current_cars().clear();
    }

    fltkview mv(0, 0, 500, 500, "fltk View");
    mv.sim = &s;

    mv.back_image = new big_image(argv[2]);
    mv.back_image_center = vec2f(-172.871f, 276.668f);//vec2f(-11.7303f, -876.804f);
    mv.back_image_scale =  0.0784584;
    mv.back_image_yscale = 1.0;


    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
