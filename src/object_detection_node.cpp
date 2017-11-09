#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <sstream>
#include <math.h>
#include "pitt_msgs/TrackedShapes.h" 	// for out message (an array of TrackedShape), it internally include TrackedShape.h
#include "pointing_gestures_object_detection/Joints.h"
#include "pointing_gestures_object_detection/PointedObjects.h"
#include <exception>

using namespace std;
using namespace pitt_msgs;
using namespace pointing_gestures_object_detection;
using namespace geometry_msgs;

//funzione per parsare un intero in stringa 
string int_to_string(int num){
	stringstream ss;
	ss<<num;
	return ss.str();
}
 
class Objects {
	
	public: 
	
		static bool myCompare(Objects op_1, Objects op_2) { 
			return (op_1.distance_from_cog < op_2.distance_from_cog); 
		}
		
		string id;
		Point centerOfGravity;
		string object_shape;
		Vector3 object_vector, plane_vector, orthogonal_object_vector;
		float dim_1, dim_2;
		float distance_from_cog;
	
		//constructor
		Objects(int _id, Point _centerOfGravity, string _object_shape, Vector3 _object_vector, Vector3 _plane_vector, Vector3 _orthogonal_object_vector, float _dim_1, float _dim_2){
		
			id = int_to_string(_id);
			centerOfGravity = _centerOfGravity;
			object_shape = _object_shape;
			object_vector = _object_vector; 						//vettore planare per darne la direzione
			plane_vector = _plane_vector;							//vettore perpendicolare al piano
			orthogonal_object_vector = _orthogonal_object_vector; 	//vettore planare perpendicolare all'atro
			dim_1 = _dim_1;
			dim_2 = _dim_2;
			distance_from_cog = 0.0;
			
		}		
};

//contains id, center of gravity, shape and dimensions of objects detected on the table
vector<Objects> objects_vector;

//contains scenes data
vector<Objects> scenes_vector;	//da riempire nella callback dell'ontologia vengono inserite solo le scene, non gli oggetti

//contiene oggetti e scene
vector<Objects> objects_and_scenes;

//contains coordinates of right elbow and right hand when user is actually pointing
Point r_hand, r_elbow;

//pointing results publisher
ros::Publisher pointed_objects_publisher;

PointedObject::Ptr pointed_object(new PointedObject);
PointedObjects::Ptr pointed_objects(new PointedObjects);

//funzione che passato un vector prende gli ultimi tre elementi che sono float
//e li usa come coordinate di un punto
Point readPointCoordinates(vector<float> _coefficients, int i){
	
	Point tmpPoint;

	tmpPoint.z = _coefficients[i];
	tmpPoint.y = _coefficients[i-1];
	tmpPoint.x = _coefficients[i-2];
	
	/*tmpPoint.z = *_coefficients.end();
	_coefficients.pop_back();

	tmpPoint.y = *_coefficients.end();
	_coefficients.pop_back();

	tmpPoint.x = *_coefficients.end();
	_coefficients.pop_back();*/

	return tmpPoint;
}

Vector3 axisVectorCalc(Point _p_1, Point _p_2){

	Vector3 tmpVector;
	
	tmpVector.x = _p_1.x - _p_2.x;
	tmpVector.y = _p_1.y - _p_2.y;
	tmpVector.z = _p_1.z - _p_2.z;

	return tmpVector;
	
}

//update objects_vector every time a trackedShapes msg is published (it also filters unknown objects)
void trackedShapesCallback(const TrackedShapesConstPtr& msgTrackedShapes){
	
	//clearing objects_vector
	objects_vector.clear();
	
	//temp vector: contains all the objects detected by the Pitt Package
	vector<TrackedShape> tmpVector(msgTrackedShapes->tracked_shapes);
	
	for(std::vector<TrackedShape>::iterator it = tmpVector.begin(); it != tmpVector.end(); ++it) {
		
		//deleting unknown objects 
		if(it->shape_tag.compare("unknown")){	
		
			Point tmpCOGPoint;
			Vector3 tmp_plane_vector, tmp_object_vector, tmp_orthogonal_object_vector;
			float dim_1, dim_2;
			
			//centro di massa dell'oggeto preso da quello calcolato sul point cloud
			tmpCOGPoint.x = it->x_pc_centroid;
			tmpCOGPoint.y = it->y_pc_centroid;
			tmpCOGPoint.z = it->z_pc_centroid;
			
			if(!it->shape_tag.compare("sphere")){
	
				//vettori entrambi non servono (quello del piano è calcolato dalla funzione apposita)
				
				//ultimo valore del vettore coefficients è il raggio della sfera
				dim_1 = dim_2 = (it->coefficients[3]);
				
			}
			else if(!it->shape_tag.compare("plane")){
				
				Point p1, p2, p3, p4;
				
				p4 = readPointCoordinates(it->coefficients, 15);
				p3 = readPointCoordinates(it->coefficients, 12);
				p2 = readPointCoordinates(it->coefficients, 9);
				p1 = readPointCoordinates(it->coefficients, 6);
				
				//scarto il valore di d dei coefficineti del piano, non mi interessa
				//it->coefficients.pop_back();			
				
				tmp_plane_vector.z = it->coefficients[2];
				//it->coefficients.pop_back();
				
				tmp_plane_vector.y = it->coefficients[1];
				//it->coefficients.pop_back();
				
				tmp_plane_vector.x = it->coefficients[0];
				//it->coefficients.pop_back();
				
				//calcolo del vettore planare su un asse del piano
				tmp_object_vector.x = p4.x -p1.x;
				tmp_object_vector.y = p4.y -p1.y;
				tmp_object_vector.z = p4.z -p1.z;
				
				//calcolo il vettore ortogonale a quello planare e a quello ortogonale al piano 
				tmp_object_vector, tmp_plane_vector, tmp_orthogonal_object_vector;
				
				tmp_orthogonal_object_vector.x = ((tmp_plane_vector.y * tmp_object_vector.z) - (tmp_plane_vector.z * tmp_object_vector.y));
				tmp_orthogonal_object_vector.y = ((tmp_plane_vector.z * tmp_object_vector.x) - (tmp_plane_vector.x * tmp_object_vector.z));
				tmp_orthogonal_object_vector.z = ((tmp_plane_vector.x * tmp_object_vector.y) - (tmp_plane_vector.y * tmp_object_vector.x));
				
				dim_1 = (sqrt(pow((p1.x - p2.x), 2.0) + 
							 pow((p1.y - p2.y), 2.0) +
							 pow((p1.z - p2.z), 2.0)))/2.0;
				
				dim_2 =  (sqrt(pow((p1.x - p4.x), 2.0) + 
							  pow((p1.y - p4.y), 2.0) +
							  pow((p1.z - p4.z), 2.0)))/2.0;	
				
			}
			else if(!it->shape_tag.compare("cone")){
			
				//height
				dim_2 =  (it->coefficients[7]);
				//it->coefficients.pop_back();
				
				//angolo in radianti per il calcolo del raggio
				float radiant = (it->coefficients[6]);
				//it->coefficients.pop_back();
				
				//radius
				dim_1 = dim_2 * tan(radiant);																													

				//mi interessa la metà dell'altezza per i calcoli dell'intorno
				dim_2 = dim_2 / 2.0;
				
				//calcolo il vettore dell'asse del cono dati un punto sull'asse e il vertice 
				Point tmpPoint1, tmpPoint2;

				tmpPoint1 = readPointCoordinates(it->coefficients, 5);
				tmpPoint2 = readPointCoordinates(it->coefficients, 2);
				
				tmp_object_vector = axisVectorCalc(tmpPoint1, tmpPoint2);
					
			}
			else if(!it->shape_tag.compare("cylinder")){
				
				
				
				//mezza altezza è quello che mi interessa per i calcoli dell'intorno 
				dim_2 =  (it->coefficients[7])/2.0;
				//it->coefficients.pop_back();
				
				//radius
				dim_1 =  (it->coefficients[6]);
				//it->coefficients.pop_back();
				
				//calcolo il vettore dell'asse del cilindro dati due punti sull'asse
				Point tmpPoint1, tmpPoint2;

				tmpPoint1 = readPointCoordinates(it->coefficients, 5);
				tmpPoint2 = readPointCoordinates(it->coefficients, 2);
				
				tmp_object_vector = axisVectorCalc(tmpPoint1, tmpPoint2);
			}
	/*
			cout<<"id : "<<it->object_id<<endl;
			cout<<"Shape : "<<it->shape_tag<<endl;
			cout<<"dim_1 : "<<dim_1<<endl;
			cout<<"dim_2 : "<<dim_2<<endl;
			cout<<"---"<<endl;
	*/	
			
			objects_vector.push_back(Objects(it->object_id, tmpCOGPoint, it->shape_tag, tmp_object_vector, tmp_plane_vector, tmp_orthogonal_object_vector, dim_1, dim_2));

			
			
		}
	}
}

std::vector<float> planeParamsVector(Point _r_hand_pose, Point _r_elbow_pose, Objects _object){

	//vettore di ritorno
	std::vector<float> plane_params;
	float a,b,c,d;
	
	if(!_object.object_shape.compare("sphere")){
	
		//piano costruito sul baricentro dell'oggetto e perpendicolare al puntamento
		a = _r_hand_pose.x - _r_elbow_pose.x;												
		b = _r_hand_pose.y - _r_elbow_pose.y;												
		c = _r_hand_pose.z - _r_elbow_pose.z;																				
		d = -(a*_object.centerOfGravity.x + b*_object.centerOfGravity.y + c*_object.centerOfGravity.z);
	
	}
	else if(!_object.object_shape.compare("plane")){
		
		//piano costruito sul piano dato direttamente 
		a = _object.plane_vector.x;
		b = _object.plane_vector.y;
		c = _object.plane_vector.z;
		d = -(a*_object.centerOfGravity.x + b*_object.centerOfGravity.y + c*_object.centerOfGravity.z);
		
	}
	else if(!_object.object_shape.compare("cone") || !_object.object_shape.compare("cylinder")){
	
		//facio di piani costruito sulla retta dell'asse
	
		//calcolo il piano perpendicolare all'asse dell'oggetto e passante per il baricentro 
		float p_a,p_b,p_c,p_d;
		p_a = _object.object_vector.x;
		p_b = _object.object_vector.y;
		p_c = _object.object_vector.z;
		p_d = -(p_a*_object.centerOfGravity.x + p_b*_object.centerOfGravity.y + p_c*_object.centerOfGravity.z);

		//retta per trovare la proiezione dei punti di mano e gomito passante per il punto della mano 
		//e parallela al vettore dell'oggetto (nella variabile p_ sta per proiezione)
		//p_hand_x = x_hand + t * p_a;
		//p_hand_y = y_hand + t * p_b;
		//p_hand_z = z_hand + t * p_c;

		//parametro della retta per trovare l'intersezione con il piano perpendicolare all'oggetto
		float t;
		//proiezione ortogonale dei punti di mano e gomito sul piano perpendicolare all'oggetto
		float p_hand_x, p_hand_y, p_hand_z;
		float p_elbow_x, p_elbow_y, p_elbow_z;

		t = -((p_a*_r_hand_pose.x + p_b*_r_hand_pose.y + p_c*_r_hand_pose.z + p_d)/
					(p_a*p_a + p_b*p_b + p_c*p_c));			

		p_hand_x = _r_hand_pose.x + t * p_a;
		p_hand_y = _r_hand_pose.y + t * p_b;
		p_hand_z = _r_hand_pose.z + t * p_c;

		t = -((p_a*_r_elbow_pose.x + p_b*_r_elbow_pose.y + p_c*_r_elbow_pose.z + p_d)/
					(p_a*p_a + p_b*p_b + p_c*p_c));	

		p_elbow_x = _r_elbow_pose.x + t * p_a;
		p_elbow_y = _r_elbow_pose.y + t * p_b;
		p_elbow_z = _r_elbow_pose.z + t * p_c;


		//trovo il vettore dati i due punti sopra calcolati
		Vector3 planeVector;
		
		planeVector.x = p_hand_x - p_elbow_x;
		planeVector.y = p_hand_y - p_elbow_y;
		planeVector.z = p_hand_z - p_elbow_z;

		//dati i due punti di mano e gomito proiettati sul piano trovo il vettore cercato perpendicolare al piano
		a = p_hand_x - p_elbow_x;
		b = p_hand_y - p_elbow_y;
		c = p_hand_z - p_elbow_z;
		d = -(a*_object.centerOfGravity.x + b*_object.centerOfGravity.y + c*_object.centerOfGravity.z);
		
	} //else if per le scene

	plane_params.push_back(a);
	plane_params.push_back(b);
	plane_params.push_back(c);
	plane_params.push_back(d);
	
	return plane_params;
}

float pointToPointDistance(Point _p_1, Point _p_2){
	
	return sqrt(pow((_p_1.x - _p_2.x), 2.0) + 
				pow((_p_1.y - _p_2.y), 2.0) +
				pow((_p_1.z - _p_2.z), 2.0));

}

//funzione che ritorna la distanza tra un punto ed una retta
//dati il punto il vettore direzione della retta ed un punto ad essa appartenente
float pointToLineDistance(Point _point, Vector3 _line_vector, Point _line_point){
	
	float a, b, c, d;
	
	//trovo il piano passante per il punto e con vettore parallelo alla linea interessata
	a = _line_vector.x;												
	b = _line_vector.y;												
	c = _line_vector.z;																				
	d = -(a*_point.x + b*_point.y + c*_point.z);
	
	//trovo il punto intersezione tra il piano appena trovato e la retta di eq parametriche
	//x = _line_point.x + t * _line_vector.x;
	//y = _line_point.y + t * _line_vector.y;
	//z = _line_point.z + t * _line_vector.z;
	
	float t = -((a*_line_point.x + b*_line_point.y + c*_line_point.z + d)/(a*_line_vector.x + b*_line_vector.y + c*_line_vector.z));
	
	Point second_point;
	second_point.x = _line_point.x + t * _line_vector.x;
	second_point.y = _line_point.y + t * _line_vector.y;
	second_point.z = _line_point.z + t * _line_vector.z;
	
	return pointToPointDistance(_point, second_point);
	
}

bool isPointing(Point _intersection, Objects _object){ 
	
	//if else sul tipo di oggetto per avere calcoli diversi
	
	//forse solo la sfera ha un calcolo diverso 
	
	//devo calcolare il secondo vettore planare nel caso di cono e cilindro
	if(!_object.object_shape.compare("sphere")){
	
		//distanza punto-punto in questo caso intersezione e baricentro
		if(pointToPointDistance(_intersection, _object.centerOfGravity) > _object.dim_1){	
			
			//cout << " distanza puntamento - baricentro : "<< pointToPointDistance(_intersection, _object.centerOfGravity) << endl;
			//cout << " dim_1 : "<< _object.dim_1 << endl;
			return false;
		}
		
		return true;
		
	}
	else if(!_object.object_shape.compare("plane") ||
			!_object.object_shape.compare("cone") ||
			!_object.object_shape.compare("cylinder")){
		
		//calcolo la distanza punto retta e vedo se è minore dei parametri trovati in precedenza 
		if(pointToLineDistance(_intersection, _object.object_vector , _object.centerOfGravity) < _object.dim_1 && 
			pointToLineDistance(_intersection, _object.orthogonal_object_vector , _object.centerOfGravity) < _object.dim_2){
				return true;
		}
		
		return false;
	}
	
	
}

// returns an ordered vector with the id of all the possible objects that user could have pointed
void pointedObjects(Point _r_hand_pose, Point _r_elbow_pose, vector<Objects> _objectsVector){

	//vector that contains all the pointed objects and distances form their center of gravity
	vector<Objects> dist_from_cog;
	
	// parametric straight bulid through elbow and hand's joints coordinates
	vector<string> pointed_ids;
	
	// cycle on every objects detected on the table
	for(std::vector<Objects>::iterator it = _objectsVector.begin(); it != _objectsVector.end(); ++it) {
		
		//plane parameters 
		float a,b,c,d;
		
		vector<float> planeParams = planeParamsVector(_r_hand_pose, _r_elbow_pose, *it);
		
		d = planeParams[3];
		//planeParams.pop_back();
		
		c = planeParams[2];
		//planeParams.pop_back();
		
		b = planeParams[1];
		//planeParams.pop_back();
		
		a = planeParams[0];
		//planeParams.pop_back();
		
		//forse esiste un costruttore più elegante
		Vector3 calculated_plane_vector;
		calculated_plane_vector.x = a;
		calculated_plane_vector.y = b;
		calculated_plane_vector.z = c;
		
		it->plane_vector = calculated_plane_vector;
		
		if(!it->object_shape.compare("cone") || !it->object_shape.compare("cylinder")){
			
			//calcolo il vettore perpendicolare a quello dell'asse e a quello del piano costruito
			it->orthogonal_object_vector.x = ((it->plane_vector.y * it->object_vector.z) - (it->plane_vector.z * it->object_vector.y));
			it->orthogonal_object_vector.y = ((it->plane_vector.z * it->object_vector.x) - (it->plane_vector.x * it->object_vector.z));
			it->orthogonal_object_vector.z = ((it->plane_vector.x * it->object_vector.y) - (it->plane_vector.y * it->object_vector.x));

		}
		
		//intersection between pointing line and plane founded
		float t = -((a*_r_elbow_pose.x + b*_r_elbow_pose.y + c*_r_elbow_pose.z + d)/
					(a*(_r_hand_pose.x - _r_elbow_pose.x) + b*(_r_hand_pose.y - _r_elbow_pose.y) + c*(_r_hand_pose.z - _r_elbow_pose.z)));
		
		//intersection point
		Point intersection;
		intersection.x = _r_elbow_pose.x + t * (_r_hand_pose.x - _r_elbow_pose.x);
		intersection.y = _r_elbow_pose.y + t * (_r_hand_pose.y - _r_elbow_pose.y);
		intersection.z = _r_elbow_pose.z + t * (_r_hand_pose.z - _r_elbow_pose.z);
		
		//check if the intersection point is actually around the object
		if(isPointing(intersection, *it)){
			
			it->distance_from_cog = pointToPointDistance(intersection, it->centerOfGravity);
			dist_from_cog.push_back(*it);
			
		}
	
	}
	
	if(!dist_from_cog.empty()) {
	
		//sorting del vettore secondo la distanza minore dal proprio centro di massa
		std::sort(dist_from_cog.begin(), dist_from_cog.end(), Objects::myCompare);

		//for per inserire nel vettore finale gli id ordinati
		for(std::vector<Objects>::iterator it = _objectsVector.begin(); it != _objectsVector.end(); ++it) {	
			
			pointed_object->object_id = it->id;
			pointed_object->distance = it->distance_from_cog;
			
			pointed_objects->pointed_objects.push_back(*pointed_object);
			
			pointed_ids.push_back(it->id);
		}

		//PUBBLICARE CON UN PUBLISHER IL VETTORE RISULTATO pointed_ids IN UN MESSAGGIO OPPRTUNO
		cout<<dist_from_cog[0].object_shape<<endl;

		pointed_objects_publisher.publish(pointed_objects);
		
	}
	else{
		cout << "Any object pointed!" << endl;
	}
	
	
}

void jointsCallback(const JointsConstPtr& msgJoints){
	
	//getting joints data
	r_hand = msgJoints->r_hand_pose;
	r_elbow = msgJoints->r_elbow_pose;
	
	objects_and_scenes.clear();
	
	//concatena i due vettori di oggetti e scene in uno unico
	objects_and_scenes.reserve( objects_vector.size() + scenes_vector.size() ); // preallocate memory
	objects_and_scenes.insert( objects_and_scenes.end(), objects_vector.begin(), objects_vector.end() );
	objects_and_scenes.insert( objects_and_scenes.end(), scenes_vector.begin(), scenes_vector.end() );
	
		
	if(!objects_and_scenes.empty()){
		pointedObjects(r_hand, r_elbow, objects_and_scenes);
	}
	
	
}



int main( int argc, char** argv )
{
	cout << "OBJECT DETECTION NODE"<< endl;
	ros::init(argc, argv, "object_detection_node");
	ros::NodeHandle node;
	ros::Rate r(6);			//frequency

	ros::Subscriber pitt_sub = node.subscribe("correct_shapes", 1, trackedShapesCallback);
	ros::Subscriber tracker_sub = node.subscribe("right_joints", 1, jointsCallback);
	pointed_objects_publisher = node.advertise<PointedObjects>("pointed_objects", 1);
	
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
}
