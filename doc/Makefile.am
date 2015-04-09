noinst_DATA = html

EXTRA_DIST = html

if HAVE_DOXYGEN
html-local: Doxyfile
	$(DOXYGEN) $(DOXYFLAGS) $<
endif