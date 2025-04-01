#ifndef INPUTREADER_H
#define INPUTREADER_H

#include <opencv2/core/types.hpp>
#include <map>
#include <string>
#include <vector>

using namespace std;

class Map;
class InputParser;

class InputReader {
    using Match = pair<int, int>;

public:
    // Constructor
    explicit InputReader(InputParser* input);

    // Getters
    int GetNumFrames() ;
    int GetInitDist() ;
    cv::Mat GetK() ;
    float GetFocalDistance();
    float getScaleDistance() const { return scaleDistance; }
    int getFrame0() const { return frame0; }
    int getFrame1() const { return frame1; }
    std::string GetImageName(int frameId) ;
    std::vector<int> GetImageBounds(cv::Mat K) ;
    std::vector<int> GetNotInitialFrames(int n) ;
    std::vector<cv::KeyPoint> GetKPs(int frameId) ;
    std::vector<cv::KeyPoint> GetUndistortedKPs(int frameId, cv::Mat K) ;
    std::vector<int> GetTrackIds(int frameId) ;
    std::vector<int> GetIndexInKfs(std::vector<int> kfs, int track) ;
    std::vector<std::tuple<int, int, int>> GetInitialPairsFromMostMatches(float dist) ;
    std::vector<std::tuple<int, int, int>> GetInitialPairsFromLeastMatches() ;
    std::vector<std::tuple<int, int, int>> GetInitialPairsFromQuartiles() ;

    // Getters (Referencias constantes)
    const std::vector<int>& getAllTracks() const { return allTracks; }
    const std::map<int, std::vector<cv::Point2f>>& getKps() const { return kps; }
    const std::map<int, std::vector<int>>& getTrackIds() const { return track_ids; }
    const std::map<int, std::string>& getImgNames() const { return img_names; }
    const std::map<int, std::map<int, float>>& getRadios() const { return radios; }
    const std::map<int, std::string>& getLabels() const { return labels; }

    // Setters
    void setKps(const std::map<int, std::vector<cv::Point2f>>& kps) { this->kps = kps; }
    void setTrackIds(const std::map<int, std::vector<int>>& trackIds) { track_ids = trackIds; }
    void setFrame0(int f) { frame0 = f; }
    void setFrame1(int f) { frame1 = f; }

    // Métodos de procesamiento
    std::map<int, Match> GetMatches(int frameId1, int frameId2) ;
    std::map<int, Match> GetMatchesSinOutliers(int frameId1, int frameId2, std::vector<int> outliers) ;
    std::map<int, cv::Point2f> GetPoints(int frameId) ;
    void showRadios();

    // Atributo de estado
    bool error;

protected:
    // Rutas de archivos
    std::string calibPath;
    std::string detectionMatchesPath;
    std::string qrMatchesPath;
    std::string imagesPath;

    // Parámetros y datos
    float scaleDistance;
    int numFrames;
    int init_min_dist;

    std::vector<int> allTracks;
    std::map<int, std::vector<cv::Point2f>> kps;
    std::map<int, std::vector<int>> track_ids;
    std::map<int, std::string> img_names;
    std::map<int, std::string> labels;
    std::map<int, std::map<int, float>> radios;

    // Variables auxiliares
    int track_cal_1;
    int track_cal_2;
    int track_val_1;
    int track_val_2;
    int frame0;
    int frame1;

    // Métodos internos
    void parseDetectionsFile(string detectionMatchesPath);
};

// ---------------------------------------------------------
// Clase InputParser
// ---------------------------------------------------------

class InputParser {
public:
    explicit InputParser(int& argc, char** argv);

    std::string getCmdOption(std::string option);
    bool cmdOptionExists(std::string option);
    void checkArgs();

    bool calibFlag, outputFlag, qrFlag, detectionsFlag, imagesFlag, scaleFlag, initFlag;

private:
    std::vector<std::string> tokens;
};

#endif // INPUTREADER_H

