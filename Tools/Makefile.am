noinst_PROGRAMS = hybrid  pc-int-test Stockholm I80 Berkeley_data

EXTRA_DIST = arcball.hpp big-image-tile.hpp

hybrid_SOURCES  = hybrid.cpp
hybrid_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
hybrid_LDFLAGS  = $(LDFLAGS)
hybrid_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

I80_SOURCES  = I80.cpp
I80_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CAIRO_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(FLTK_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(MAGICKPP_CXXFLAGS)
I80_LDFLAGS  = $(LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS) $(MAGICKPP_LDFLAGS)
I80_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

# traffic_viz_SOURCES  = traffic_viz.cpp
# traffic_viz_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CAIRO_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(FLTK_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(MAGICKPP_CXXFLAGS)
# traffic_viz_LDFLAGS  = $(LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS) $(MAGICKPP_LDFLAGS)
# traffic_viz_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

# BF_reconstruct_SOURCES  = BF_reconstruct.cpp matrix.cpp bayesianFilter.cpp
# BF_reconstruct_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CAIRO_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(FLTK_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(MAGICKPP_CXXFLAGS)
# BF_reconstruct_LDFLAGS  = $(LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS) $(MAGICKPP_LDFLAGS) -llapack
# BF_reconstruct_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

# new_enkf_reconstruct_SOURCES  = new_enkf_reconstruct.cpp matrix.cpp bayesianFilter.cpp ENKF_traffic_parser.cpp ENKF_ensemble.cpp
# new_enkf_reconstruct_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CAIRO_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(FLTK_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(MAGICKPP_CXXFLAGS)
# new_enkf_reconstruct_LDFLAGS  = $(LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS) $(MAGICKPP_LDFLAGS) -llapack
# new_enkf_reconstruct_LDADD    = $(top_builddir)/libhybrid/libhybrid.la


Stockholm_SOURCES  = Stockholm.cpp
Stockholm_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CAIRO_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(FLTK_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(MAGICKPP_CXXFLAGS)
Stockholm_LDFLAGS  = $(LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS) $(MAGICKPP_LDFLAGS)

Stockholm_LDADD    = $(top_builddir)/libhybrid/libhybrid.la


Berkeley_data_SOURCES  = Berkeley_data.cpp
Berkeley_data_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CAIRO_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(FLTK_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(MAGICKPP_CXXFLAGS)
Berkeley_data_LDFLAGS  = $(LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS) $(MAGICKPP_LDFLAGS)

Berkeley_data_LDADD    = $(top_builddir)/libhybrid/libhybrid.la


# Routing_SOURCES  = Routing.cpp stoch_graph/stoch_graph.cpp
# Routing_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CAIRO_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(FLTK_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(MAGICKPP_CXXFLAGS)
# Routing_LDFLAGS  = $(LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS) $(MAGICKPP_LDFLAGS)

# Routing_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

# LastMin_SOURCES  = LastMin.cpp stoch_graph/stoch_graph.cpp
# LastMin_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CAIRO_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(FLTK_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir) $(MAGICKPP_CXXFLAGS)
# LastMin_LDFLAGS  = $(LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS) $(MAGICKPP_LDFLAGS)

# LastMin_LDADD    = $(top_builddir)/libhybrid/libhybrid.la



# ih_riemann_test_SOURCES  = ih-riemann-test.cpp
# ih_riemann_test_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
# ih_riemann_test_LDFLAGS  = $(LDFLAGS)
# ih_riemann_test_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

pc_int_test_SOURCES  = pc-int-test.cpp
pc_int_test_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
pc_int_test_LDFLAGS  = $(LDFLAGS)
pc_int_test_LDADD    = $(top_builddir)/libhybrid/libhybrid.la


# if DO_VISUAL
# if DO_IMAGE
# dump_to_png_SOURCES  = dump-to-png.cpp
# dump_to_png_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
# dump_to_png_LDFLAGS  = $(LDFLAGS)
# dump_to_png_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

# noinst_PROGRAMS += hybrid-viewer dump-to-png

# hybrid_viewer_SOURCES = hybrid-viewer.cpp
# hybrid_viewer_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(FLTK_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir)
# hybrid_viewer_LDFLAGS  = $(LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS)
# hybrid_viewer_LDADD    = $(top_builddir)/libhybrid/libhybrid.la  -lroad #$(LIBROAD_LIBS)/libroad.la

# noinst_PROGRAMS += traffic-2d

# traffic_2d_SOURCES  = traffic-2d.cpp
# traffic_2d_CPPFLAGS = $(LIBROAD_CFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CAIRO_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(MAGICKPP_CXXFLAGS) $(CAIRO_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
# traffic_2d_LDFLAGS  = $(BOOST_FILESYSTEM_LIBS) $(CAIRO_LIBS) $(MAGICKPP_LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS) $(LDFLAGS)
# traffic_2d_LDADD    = $(BOOST_THREAD_LIBS) $(BOOST_REGEX_LIBS) $(top_builddir)/libhybrid/libhybrid.la -lroad  #$(LIBROAD_LIBS)/libroad.la
# endif
# endif
