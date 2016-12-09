#ifndef CLI_HPP
#define CLI_HPP

#include <string>
#include <iostream>

class ParamBuilder {

    std::string inputFile = "";
    std::string outputFile = "";
    double groundWidth = 0.8;
    double groundHeight = 0.5;
    double clusterResolution = 0.02;
    double highResolution = 0.2;
    double lowResolution = 0.05;
    double minHeight = 2.0;
    double maxHeight = 0.6;

    bool ghosts = false;
    bool debug = false;

    const char* usage = "Használat: szellem -i [bemenet] -o [kimenet] [paraméterek]\n"
            "\t\n"
            "\t-i\tBemeneti fájl, vagy mappa.\n"
            "\t-o \tKimeneti fájl, vagy mappa. Mappa esetén a kimenet a bemeneti név \n"
            "\t\t'out' előtaggal.\n"
            "\t\n"
            "\t-a\tKimenetként generál egy 'szellem' előtagú pontfelhőt, mely a \n"
            "\t\tvégeredményből kimaradt pontokat tartalmazza.\n"
            "\t-d\tFutás közben részletes információt ad a lépések végrehajtásához \n"
            "\t\tszükséges időről.\n"
            "\t-h\tEnnek a súgónak a megjelenítése\n"
            "\t\n"
            "\t-tw\tA cella alapú talajeltávolításnál használt cella szélessége.\n"
            "\t-th\tA cella alapú talajeltávolításnál megengedett maximális magasság.\n"
            "\t-er\tAz első euklideszi távolságon alapuló klaszterezés sugara.\n"
            "\t-mr\tA magas objektumok szűrése előtti klaszterezés sugara.\n"
            "\t-mm\tA magas objektumok szűréséhez használt minimális méret.\n"
            "\t-ar\tAz alacsony objektumok szűrése előtti klaszterezés sugara.\n"
            "\t-am\tAz alacsony objektumok szűréséhez használt maximális méret.\n"
            "\t\n"
            "Példa: \tszellem -i bemenet.laz -o kimenet.laz -tw 0.8 -th 0.5 -er 0.02 \n"
            "\t\t-mr 0.2 -mm 2.0 -ar 0.05 -am 0.5";

public:

    ParamBuilder(int argc, char *argv[]){
        if(argc == 2){
            std::string in = argv[1];
            if(in.compare("-h") == 0){
                std::cout << usage << std::endl;
            }
        } else {
            int i = 1;
            while(i < argc){
                std::string in = argv[i];
                if(in.compare("-i") == 0){
                    inputFile = argv[i+1];
                    i += 2;
                } else if(in.compare("-o") == 0){
                    outputFile = argv[i+1];
                    i += 2;
                } else if(in.compare("-a") == 0){
                    ghosts = true;
                    i += 1;
                } else if(in.compare("-d") == 0){
                    debug = true;
                    i += 1;
                } else if(in.compare("-tw") == 0){
                    groundWidth = std::stod(argv[i+1]);
                    i += 2;
                } else if(in.compare("-th") == 0){
                    groundHeight = std::stod(argv[i+1]);
                    i += 2;
                } else if(in.compare("-er") == 0){
                    clusterResolution = std::stod(argv[i+1]);
                    i += 2;
                } else if(in.compare("-mr") == 0){
                    highResolution = std::stod(argv[i+1]);
                    i += 2;
                } else if(in.compare("-mm") == 0){
                    minHeight = std::stod(argv[i+1]);
                    i += 2;
                } else if(in.compare("-ar") == 0){
                    lowResolution = std::stod(argv[i+1]);
                    i += 2;
                } else if(in.compare("-am") == 0){
                    maxHeight = std::stod(argv[i+1]);
                    i += 2;
                } else {
                    i++;
                }
            }
        }
    }

    bool isValid(){
        if(inputFile.compare("") == 0){
            std::cout << "Hiányzó bemenet." << std::endl;
            return false;
        }
        if(outputFile.compare("") == 0){
            std::cout << "Hiányzó kimenet." << std::endl;
            return false;
        }
        return true;
    }

    const std::string &getInputFile() const {
        return inputFile;
    }

    const std::string &getOutputFile() const {
        return outputFile;
    }

    double getGroundWidth() const {
        return groundWidth;
    }

    double getGroundHeight() const {
        return groundHeight;
    }

    double getClusterResolution() const {
        return clusterResolution;
    }

    double getHighResolution() const {
        return highResolution;
    }

    double getLowResolution() const {
        return lowResolution;
    }

    double getMinHeight() const {
        return minHeight;
    }

    double getMaxHeight() const {
        return maxHeight;
    }

    bool isGhosts() const {
        return ghosts;
    }

    bool isDebug() const {
        return debug;
    }


};

#endif //CLI_HPP
