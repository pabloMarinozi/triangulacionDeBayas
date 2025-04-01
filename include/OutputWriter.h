#ifndef OUTPUTWRITER_H
#define OUTPUTWRITER_H

#include <InputReader.h>
#include <MapManager.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <map>
#include <string>
#include <vector>

using namespace std;

class Map;

class OutputWriter {
public:
	OutputWriter(InputParser* input, InputReader* mpInputReader,int f0, int f1);
  void calcularReproyecciones(MapManager* mpMapManager, InputReader* mpInputReader, bool calcularNormal);
  void guardarTriangulacion(string filename, bool escala);
  void graficarReproyecciones(InputReader* mpInputReader, bool graficarNoVisibles, bool graficarNormal);
	const string& getStrOutputPath() const {return strOutputPath;}
  //void guardarOutliers(map<int, float> outliers_prop);

protected:
	string strOutputPath;
  map<int, map<int, float> > errors_map;  //r
  map<int, string> labels; //r
	map<int, map<int, cv::Point2f> > rep_map; //r //i
	map<int, map<int, cv::Point2f> > normal_points_map; //e
	map<int, vector<float> > vols_rep_map; //v
	map<int, vector<float> > vols_real_map; //v
	map<int, map<int, float> > radios; //r
	map<int, string> all_img_names; //i
	map<int, string> img_names; //i
	map<int, cv::Mat> imgs; //i
	vector<long unsigned int> allKfIds; 
  map<int, cv::Point3d> mps;
  map<int, cv::Point3d> cams;
  map<int, vector<cv::Point2f> > kps;
  map<int, vector<int> > track_ids;
  float f;

private:
    string generarContenidoCSV(bool escala);


};
#endif // OUTPUTWRITER_H;

