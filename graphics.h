
// graphic function examples

void draw_plane(double Px, double Py, double Pz, double yaw, double pitch, 
	double roll, double scale);

void draw_box(double Px, double Py, double Pz, double yaw, double pitch, 
   double roll, double Lx, double Ly, double Lz, double R, double G, double B);

void set_view();
// change the eye_point using the keyboard				  
				  
// new functions /////////////
				  
void draw_line(double *x, double *y, double *z, double *R, double *G, 
		double *B, int n);
// x[n] - 1D array holding x components of the vertices
// y[n] - 1D array holding y components of the vertices
// z[n] - 1D array holding z components of the vertices
// R[n] - 1D array holding Red colour components (0<=R[i]<=1) of the vertices
// G[n] - 1D array holding Green colour components (0<=G[i]<=1) of the vertices
// B[n] - 1D array holding Blue colour components (0<=B[i]<=1) of the vertices
// n - number of vertices in the line

void draw_points(double *x, double *y, double *z, double *R, double *G, 
		double *B, int n);
// x[n] - 1D array holding x components of the vertices
// y[n] - 1D array holding y components of the vertices
// z[n] - 1D array holding z components of the vertices
// R[n] - 1D array holding Red colour components (0<=R[i]<=1) of the vertices
// G[n] - 1D array holding Green colour components (0<=G[i]<=1) of the vertices
// B[n] - 1D array holding Blue colour components (0<=B[i]<=1) of the vertices
// n - number of vertices in the line

void draw_polygon(double *x, double *y, double *z, double R, double G, 
		double B, int n);
// draw a polygon surface (assumes the vertices are in a 2D plane)
// x[n] - 1D array holding x components of the vertices
// y[n] - 1D array holding y components of the vertices
// z[n] - 1D array holding z components of the vertices
// n - number of vertices in the line 
// assume curve is closed -> x[n-1]=x[0], y[n-1]=y[0], z[n-1]=z[0]

void draw_circle(double x, double y, double z, int n, double radius, 
		double R, double G, double B, int flag = 1);
// draw a circle centered at (x,y,z) with n line segments and 
// colour (R,G,B), flag: 
// 1 - X-Y coordinates
// 2 - X-Z coordinates
// 3 - Y-Z coordinates

void draw_disc(double x, double y, double z, double radius, int n,
		double R, double G, double B, int flag = 1);
// draw a disc centered at (x,y,z) with n triangle segments and 
// colour (R,G,B), flag: 
// 1 - X-Y coordinates
// 2 - X-Z coordinates
// 3 - Y-Z coordinates

void draw_rectangle(double x, double y, double z, double theta, 
	double width, double height, double R, double G, double B, 
	int flag = 1);
// draw a rectangle centered at (x,y,z) and angle theta with 
// colour (R,G,B), flag: 
// 1 - X-Y coordinates (width-height)
// 2 - X-Z coordinates (width-height) 
// 3 - Y-Z coordinates (width-height)

void colour_map(double x, double y, double z, double width, 
		double height, double **R, double **G, double **B, 
		int ni, int nj, int flag = 1);
// draw a rectangle map centered at (x,y,z) with colour
// at the vertices given by 2D arrays R[i][j], G[i][j], B[i][j]
// flag: 
// 1 - X-Y (i-j) coordinates
// 2 - X-Z (i-j) coordinates
// 3 - Y-Z (i-j) coordinates

	
