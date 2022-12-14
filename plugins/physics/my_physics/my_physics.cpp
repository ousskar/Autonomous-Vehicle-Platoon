/*
 * File:          
 * Date:          
 * Description:   
 * Author:        
 * Modifications: 
 */

#include <ode/ode.h>
#include <plugins/physics.h>

/*
 * Note: This plugin will become operational only after it was compiled and associated with the current world (.wbt).
 * To associate this plugin with the world follow these steps:
 *  1. In the Scene Tree, expand the "WorldInfo" node and select its "physics" field
 *  2. Then hit the [Select] button at the bottom of the Scene Tree
 *  3. In the list choose the name of this plugin (same as this file without the extention)
 *  4. Then save the .wbt by hitting the "Save" button in the toolbar of the 3D view
 *  5. Then revert the simulation: the plugin should now load and execute with the current simulation
 */

void webots_physics_init() {
  /*
   * Get ODE object from the .wbt model, e.g.
   *   dBodyID body1 = dWebotsGetBodyFromDEF("MY_ROBOT");
   *   dBodyID body2 = dWebotsGetBodyFromDEF("MY_SERVO");
   *   dGeomID geom2 = dWebotsGetGeomFromDEF("MY_SERVO");
   * If an object is not found in the .wbt world, the function returns NULL.
   * Your code should correcly handle the NULL cases because otherwise a segmentation fault will crash Webots.
   *
   * This function is also often used to add joints to the simulation, e.g.
   *   dWorldID world = dBodyGetWorld(body1);
   *   dJointID joint = dJointCreateBall(world, 0);
   *   dJointAttach(joint, body1, body2);
   *   ...
   */
}

void webots_physics_step() {
  /*
   * Do here what needs to be done at every time step, e.g. add forces to bodies
   *   dBodyAddForce(body1, f[0], f[1], f[2]);
   *   ...
   */
}

void webots_physics_draw(int pass, const char *view) {
  /*
   * This function can optionally be used to add OpenGL graphics to the 3D view, e.g.
   * if (pass == 1 && view == NULL) {
   *   // setup draw style
   *   glDisable(GL_LIGHTING);
   *   glLineWidth(2);
   * 
   *   // draw a yellow line
   *   glBegin(GL_LINES);
   *   glColor3f(1, 1, 0);
   *   glVertex3f(0, 0, 0);
   *   glVertex3f(0, 1, 0);
   *   glEnd();
   * }
   *
   * Note 1: 2 passes per rendering are performed, one before the 2D view rendering (pass = 0), and one after (pass = 1).
   *         You can handle that using the "pass" parameter.
   * Note 2: It is possible to draw in the robot cameras using the view parameter (it contains the Robot::name field)
   */
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  /*
   * This function needs to be implemented if you want to overide Webots collision detection.
   * It must return 1 if the collision was handled and 0 otherwise. 
   * Note that contact joints should be added to the contact_joint_group which can change over the time, e.g.
   *   n = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
   *   dJointGroupID contact_joint_group = dWebotsGetContactJointGroup();
   *   dWorldID world = dBodyGetWorld(body1);
   *   ...
   *   dJointCreateContact(world, contact_joint_group, &contact[i])
   *   dJointAttach(contact_joint, body1, body2);
   *   ...
   */
  return 0;
}

void webots_physics_cleanup() {
  /*
   * Here you need to free any memory you allocated in above, close files, etc.
   * You do not need to free any ODE object, they will be freed by Webots.
   */
}
