/*
 * Compute lookup table of cos and sin values forming a cirle
 *
 * Notes:
 *    It is the responsibility of the caller to free these tables
 *    The size of the table is (n+1) to form a connected loop
 *    The last entry is exactly the same as the first
 *    The sign of n can be flipped to get the reverse loop
 */
#ifndef GLUT_GEOMETRY_H
#define GLUT_GEOMETRY_H

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl.h>
#else
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#endif


void fghCircleTable(double **sint,double **cost,const int n);

void glutSolidCube(GLdouble dSize);
void glutWireCube(GLdouble dSize);
void glutSolidSphere(GLdouble radius, GLint slices, GLint stacks);
void glutWireSphere(GLdouble radius, GLint slices, GLint stacks);
void glutSolidCone( GLdouble base, GLdouble z_min, GLdouble z_max, GLint slices, GLint stacks );
void glutWireCone( GLdouble base, GLdouble z_min, GLdouble z_max, GLint slices, GLint stacks);
void glutSolidCylinder(GLdouble radius, GLdouble height, GLint slices, GLint stacks);
void glutWireCylinder(GLdouble radius, GLdouble height, GLint slices, GLint stacks);
void glutWireTorus( GLdouble dInnerRadius, GLdouble dOuterRadius, GLint nSides, GLint nRings );
void glutSolidTorus( GLdouble dInnerRadius, GLdouble dOuterRadius, GLint nSides, GLint nRings );

#endif  // GLUT_GEOMETRY_H
/*** END OF FILE ***/

