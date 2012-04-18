#include <vamp-hostsdk/PluginHostAdapter.h>
#include <vamp-hostsdk/PluginInputDomainAdapter.h>
#include <vamp-hostsdk/PluginLoader.h>

#include <iostream>
#include <fstream>
#include <set>
#include <sndfile.h>
#include <signal.h>
#include <deque>

#include <cstring>
#include <cstdlib>

#include "system.h"

#include <portaudio.h>

#include <cmath>

using namespace std;

using Vamp::Plugin;
using Vamp::PluginHostAdapter;
using Vamp::RealTime;
using Vamp::HostExt::PluginLoader;
using Vamp::HostExt::PluginWrapper;
using Vamp::HostExt::PluginInputDomainAdapter;

#define HOST_VERSION "0.1"
#define SAMPLE_RATE  (44100)
#define PA_SAMPLE_TYPE  paFloat32

enum Verbosity {
    PluginIds,
    PluginOutputIds,
    PluginInformation,
    PluginInformationDetailed
};

void printFeatures(int, int, int, Plugin::FeatureSet, ofstream *, bool frames);
void transformInput(float *, size_t);
void fft(unsigned int, bool, double *, double *, double *, double *);
void printPluginPath(bool verbose);
void printPluginCategoryList();
void enumeratePlugins(Verbosity);
void listPluginsInLibrary(string soname);
int runPlugin(string myname, string soname, string id, string output,
              int outputNo, bool frames, PaStreamParameters inputParameters);

bool interruptFlag = false;

void interrupt (int param) { interruptFlag = true; }

void usage(const char *name)
{
    cerr << "\n"
         << name << ": A command-line host for live Vamp plugin processing.\n\n"
        "Written by Chris Baume (chris.baume@bbc.co.uk).\n"
        "Copyright 2012 British Broadcasting Corp.\n\n"
        "Usage:\n\n"
        "  " << name << " [-s] pluginlibrary[." << PLUGIN_SUFFIX << "]:plugin[:output]\n"
        "  " << name << " [-s] pluginlibrary[." << PLUGIN_SUFFIX << "]:plugin [outputno]\n\n"
        "    -- Load plugin id \"plugin\" from \"pluginlibrary\" and run it on the\n"
        "       audio data from the default mic, retrieving the named \"output\", or output\n"
        "       number \"outputno\" (the first output by default) and dumping it to\n"
        "       standard output.\n\n"
        "       \"pluginlibrary\" should be a library name, not a file path; the\n"
        "       standard Vamp library search path will be used to locate it.  If\n"
        "       a file path is supplied, the directory part(s) will be ignored.\n\n"
        "       If the -s option is given, results will be labelled with the audio\n"
        "       sample frame at which they occur. Otherwise, they will be labelled\n"
        "       with time in seconds.\n\n"
        "  " << name << " -l\n"
        "  " << name << " --list\n\n"
        "    -- List the plugin libraries and Vamp plugins in the library search path\n"
        "       in a verbose human-readable format.\n\n"
        "  " << name << " --list-full\n\n"
        "    -- List all data reported by all the Vamp plugins in the library search\n"
        "       path in a very verbose human-readable format.\n\n"
        "  " << name << " --list-ids\n\n"
        "    -- List the plugins in the search path in a terse machine-readable format,\n"
        "       in the form vamp:soname:identifier.\n\n"
        "  " << name << " --list-outputs\n\n"
        "    -- List the outputs for plugins in the search path in a machine-readable\n"
        "       format, in the form vamp:soname:identifier:output.\n\n"
        "  " << name << " --list-by-category\n\n"
        "    -- List the plugins as a plugin index by category, in a machine-readable\n"
        "       format.  The format may change in future releases.\n\n"
        "  " << name << " -p\n\n"
        "    -- Print out the Vamp library search path.\n\n"
        "  " << name << " -v\n\n"
        "    -- Display version information only.\n"
         << endl;
    exit(2);
}

int throwError(PaError err)
{
    Pa_Terminate();
    if( err != paNoError )
    {
        fprintf( stderr, "An error occured while using the portaudio stream\n" );
        fprintf( stderr, "Error number: %d\n", err );
        fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( err ) );
        err = 1;          /* Always return 0 or 1, but no other return codes. */
    }
    return err;
}

int main(int argc, char **argv)
{
	signal(SIGINT, interrupt);

	// Portaudio code
    PaStreamParameters  inputParameters;
    PaError             err = paNoError;

    // Initialise PortAudio
    err = Pa_Initialize();
    if( err != paNoError ) return throwError(err);

    // Choose sound card
    inputParameters.device = Pa_GetDefaultInputDevice(); /* default input device */
    if (inputParameters.device == paNoDevice) {
        fprintf(stderr,"Error: No default input device.\n");
        return throwError(err);
    }
    inputParameters.channelCount = 1;
    inputParameters.sampleFormat = PA_SAMPLE_TYPE;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo( inputParameters.device )->defaultLowInputLatency;
    inputParameters.hostApiSpecificStreamInfo = NULL;

    // Vamp code
    char *scooter = argv[0];
    char *name = 0;
    while (scooter && *scooter) {
        if (*scooter == '/' || *scooter == '\\') name = ++scooter;
        else ++scooter;
    }
    if (!name || !*name) name = argv[0];

    // Display usage if not enough args
    if (argc < 2) usage(name);

    // Process simple arguments (version, list plugins etc)
    if (argc == 2) {

        if (!strcmp(argv[1], "-v")) {

            cout << "Simple Vamp plugin host version: " << HOST_VERSION << endl
                 << "Vamp API version: " << VAMP_API_VERSION << endl
                 << "Vamp SDK version: " << VAMP_SDK_VERSION << endl;
            return 0;

        } else if (!strcmp(argv[1], "-l") || !strcmp(argv[1], "--list")) {

            printPluginPath(true);
            enumeratePlugins(PluginInformation);
            return 0;

        } else if (!strcmp(argv[1], "--list-full")) {

            enumeratePlugins(PluginInformationDetailed);
            return 0;

        } else if (!strcmp(argv[1], "-p")) {

            printPluginPath(false);
            return 0;

        } else if (!strcmp(argv[1], "--list-ids")) {

            enumeratePlugins(PluginIds);
            return 0;

        } else if (!strcmp(argv[1], "--list-outputs")) {

            enumeratePlugins(PluginOutputIds);
            return 0;

        } else if (!strcmp(argv[1], "--list-by-category")) {

            printPluginCategoryList();
            return 0;

        }
    }

    // if first argument is -s, display sample frame
    bool useFrames = false;
    int base = 1;
    if (!strcmp(argv[1], "-s")) {
        useFrames = true;
        base = 2;
    }

    // read in arguments
    string soname = argv[base];
    string plugid = "";
    string output = "";
    int outputNo = -1;

    if (argc >= base+2) {

        int idx = base+1;

        if (isdigit(*argv[idx])) {
            outputNo = atoi(argv[idx++]);
        }
    }

    // separate plugin id from library name
    string::size_type sep = soname.find(':');
    if (sep != string::npos) {
        plugid = soname.substr(sep + 1);
        soname = soname.substr(0, sep);

        sep = plugid.find(':');
        if (sep != string::npos) {
            output = plugid.substr(sep + 1);
            plugid = plugid.substr(0, sep);
        }
    }

    // if plugin name invalid, display usage
    if (plugid == "") {
        usage(name);
    }
    if (output != "" && outputNo != -1) {
        usage(name);
    }
    if (output == "" && outputNo == -1) {
        outputNo = 0;
    }

    int result = runPlugin(name, soname, plugid, output, outputNo, useFrames, inputParameters);

    Pa_Terminate();

    return result;
}

int runPlugin(string myname, string soname, string id,
              string output, int outputNo, bool useFrames, PaStreamParameters inputParameters)
{
    float				*recordedSamples;
    float				*fifo;
    PaStream*           stream;
    PaError             err = paNoError;
    int					elapsed = 0;
    int					returnValue = 1;
    RealTime			rt;
    PluginWrapper		*wrapper = 0;
    RealTime			adjustment = RealTime::zeroTime;
    PluginLoader		*loader = PluginLoader::getInstance();
    PluginLoader::PluginKey key = loader->composePluginKey(soname, id);

    // load plugin
    Plugin *plugin = loader->loadPlugin(key, SAMPLE_RATE, PluginLoader::ADAPT_ALL_SAFE);
    if (!plugin) {
        cerr << myname << ": ERROR: Failed to load plugin \"" << id
             << "\" from library \"" << soname << "\"" << endl;
        return 1;
    }

    // Find block/step size
    int blockSize = plugin->getPreferredBlockSize();
    int stepSize = plugin->getPreferredStepSize();

    if (blockSize == 0) {
        blockSize = 1024;
    }
    if (stepSize == 0) {
        if (plugin->getInputDomain() == Plugin::FrequencyDomain) {
            stepSize = blockSize/2;
        } else {
            stepSize = blockSize;
        }
    } else if (stepSize > blockSize) {
        cerr << "WARNING: stepSize " << stepSize << " > blockSize " << blockSize << ", resetting blockSize to ";
        if (plugin->getInputDomain() == Plugin::FrequencyDomain) {
            blockSize = stepSize * 2;
        } else {
            blockSize = stepSize;
        }
        cerr << blockSize << endl;
    }

    // set up port audio
    fifo = new float[blockSize]();
    recordedSamples = new float[stepSize]();

    ofstream *out = 0;
    cerr << "Running plugin: \"" << plugin->getIdentifier() << "\"..." << endl;
    cerr << "Using block size = " << blockSize << ", step size = " << stepSize << endl;

    // display output name
    Plugin::OutputList outputs = plugin->getOutputDescriptors();
    Plugin::OutputDescriptor od;

    if (outputs.empty()) {
    	cerr << "ERROR: Plugin has no outputs!" << endl;
    	goto done;
    }

    if (outputNo < 0) {

        for (size_t oi = 0; oi < outputs.size(); ++oi) {
            if (outputs[oi].identifier == output) {
                outputNo = oi;
                break;
            }
        }

        if (outputNo < 0) {
            cerr << "ERROR: Non-existent output \"" << output << "\" requested" << endl;
            goto done;
        }

    } else {

        if (int(outputs.size()) <= outputNo) {
            cerr << "ERROR: Output " << outputNo << " requested, but plugin has only " << outputs.size() << " output(s)" << endl;
            goto done;
        }
    }

    od = outputs[outputNo];
    cerr << "Output is: \"" << od.identifier << "\"" << endl;

    // Initialise plugin
    if (!plugin->initialise(1, stepSize, blockSize)) {
        cerr << "ERROR: Plugin initialise (stepSize = " << stepSize << ", blockSize = "
             << blockSize << ") failed." << endl;
        goto done;
    }

    // Compensate timestamp if in freq domain
    wrapper = dynamic_cast<PluginWrapper *>(plugin);
    if (wrapper) {
        PluginInputDomainAdapter *ida =  wrapper->getWrapper<PluginInputDomainAdapter>();
        if (ida) adjustment = ida->getTimestampAdjustment();
    }

    // Open portaudio stream
    err = Pa_OpenStream(
    		&stream,
    		&inputParameters,
    		NULL,
    		SAMPLE_RATE,
    		stepSize,
    		paClipOff,
    		NULL,
    		NULL );
    if( err != paNoError ) throwError(err);

    // Start the audio stream
    err = Pa_StartStream( stream );
    if( err != paNoError ) throwError(err);
//    printf("Now recording!!\n"); fflush(stdout);

    // do until interruptFlag is true
    while (1)
    {
    	// read step of audio data
    	err = Pa_ReadStream( stream, recordedSamples, stepSize );
    	if( err != paNoError ) throwError(err);

    	// shift buffer along a step size
    	for (int i=stepSize; i<blockSize; i++) fifo[i-stepSize] = fifo[i];

    	// add new step onto end
    	for (int i=0; i<stepSize; i++) fifo[blockSize-stepSize+i] = recordedSamples[i];

    	// process and print features
    	rt = RealTime::frame2RealTime(elapsed*stepSize, SAMPLE_RATE);
    	printFeatures(RealTime::realTime2Frame(rt + adjustment, SAMPLE_RATE), SAMPLE_RATE, outputNo, plugin->process(&fifo, rt), out, useFrames);

    	// note number of blocks processed
    	elapsed++;

    	// break out of loop
    	if (interruptFlag) break;
    }

    // stop the audio stream
    err = Pa_CloseStream( stream );
    if( err != paNoError ) throwError(err);

    // clean up variables
    delete [] recordedSamples;
    delete [] fifo;

    returnValue = 0;

done:
    delete plugin;
    return returnValue;
}

void
printFeatures(int frame, int sr, int output,
              Plugin::FeatureSet features, ofstream *out, bool useFrames)
{
    for (unsigned int i = 0; i < features[output].size(); ++i) {

        if (useFrames) {

            int displayFrame = frame;

            if (features[output][i].hasTimestamp) {
                displayFrame = RealTime::realTime2Frame
                    (features[output][i].timestamp, sr);
            }

            (out ? *out : cout) << displayFrame;

            if (features[output][i].hasDuration) {
                displayFrame = RealTime::realTime2Frame
                    (features[output][i].duration, sr);
                (out ? *out : cout) << "," << displayFrame;
            }

            (out ? *out : cout)  << ":";

        } else {

            RealTime rt = RealTime::frame2RealTime(frame, sr);

            if (features[output][i].hasTimestamp) {
                rt = features[output][i].timestamp;
            }

            (out ? *out : cout) << rt.toString();

            if (features[output][i].hasDuration) {
                rt = features[output][i].duration;
                (out ? *out : cout) << "," << rt.toString();
            }

            (out ? *out : cout) << ":";
        }

        for (unsigned int j = 0; j < features[output][i].values.size(); ++j) {
            (out ? *out : cout) << " " << features[output][i].values[j];
        }

        (out ? *out : cout) << endl;
    }
}

void
printPluginPath(bool verbose)
{
    if (verbose) {
        cout << "\nVamp plugin search path: ";
    }

    vector<string> path = PluginHostAdapter::getPluginPath();
    for (size_t i = 0; i < path.size(); ++i) {
        if (verbose) {
            cout << "[" << path[i] << "]";
        } else {
            cout << path[i] << endl;
        }
    }

    if (verbose) cout << endl;
}

static
string
header(string text, int level)
{
    string out = '\n' + text + '\n';
    for (size_t i = 0; i < text.length(); ++i) {
        out += (level == 1 ? '=' : level == 2 ? '-' : '~');
    }
    out += '\n';
    return out;
}

void
enumeratePlugins(Verbosity verbosity)
{
    PluginLoader *loader = PluginLoader::getInstance();

    if (verbosity == PluginInformation) {
        cout << "\nVamp plugin libraries found in search path:" << endl;
    }

    vector<PluginLoader::PluginKey> plugins = loader->listPlugins();
    typedef multimap<string, PluginLoader::PluginKey>
        LibraryMap;
    LibraryMap libraryMap;

    for (size_t i = 0; i < plugins.size(); ++i) {
        string path = loader->getLibraryPathForPlugin(plugins[i]);
        libraryMap.insert(LibraryMap::value_type(path, plugins[i]));
    }

    string prevPath = "";
    int index = 0;

    for (LibraryMap::iterator i = libraryMap.begin();
         i != libraryMap.end(); ++i) {
        
        string path = i->first;
        PluginLoader::PluginKey key = i->second;

        if (path != prevPath) {
            prevPath = path;
            index = 0;
            if (verbosity == PluginInformation) {
                cout << "\n  " << path << ":" << endl;
            } else if (verbosity == PluginInformationDetailed) {
                string::size_type ki = i->second.find(':');
                string text = "Library \"" + i->second.substr(0, ki) + "\"";
                cout << "\n" << header(text, 1);
            }
        }

        Plugin *plugin = loader->loadPlugin(key, 48000);
        if (plugin) {

            char c = char('A' + index);
            if (c > 'Z') c = char('a' + (index - 26));

            PluginLoader::PluginCategoryHierarchy category =
                loader->getPluginCategory(key);
            string catstr;
            if (!category.empty()) {
                for (size_t ci = 0; ci < category.size(); ++ci) {
                    if (ci > 0) catstr += " > ";
                        catstr += category[ci];
                }
            }

            if (verbosity == PluginInformation) {

                cout << "    [" << c << "] [v"
                     << plugin->getVampApiVersion() << "] "
                     << plugin->getName() << ", \""
                     << plugin->getIdentifier() << "\"" << " ["
                     << plugin->getMaker() << "]" << endl;
                
                if (catstr != "") {
                    cout << "       > " << catstr << endl;
                }

                if (plugin->getDescription() != "") {
                    cout << "        - " << plugin->getDescription() << endl;
                }

            } else if (verbosity == PluginInformationDetailed) {

                cout << header(plugin->getName(), 2);
                cout << " - Identifier:         "
                     << key << endl;
                cout << " - Plugin Version:     " 
                     << plugin->getPluginVersion() << endl;
                cout << " - Vamp API Version:   "
                     << plugin->getVampApiVersion() << endl;
                cout << " - Maker:              \""
                     << plugin->getMaker() << "\"" << endl;
                cout << " - Copyright:          \""
                     << plugin->getCopyright() << "\"" << endl;
                cout << " - Description:        \""
                     << plugin->getDescription() << "\"" << endl;
                cout << " - Input Domain:       "
                     << (plugin->getInputDomain() == Vamp::Plugin::TimeDomain ?
                         "Time Domain" : "Frequency Domain") << endl;
                cout << " - Default Step Size:  " 
                     << plugin->getPreferredStepSize() << endl;
                cout << " - Default Block Size: " 
                     << plugin->getPreferredBlockSize() << endl;
                cout << " - Minimum Channels:   " 
                     << plugin->getMinChannelCount() << endl;
                cout << " - Maximum Channels:   " 
                     << plugin->getMaxChannelCount() << endl;

            } else if (verbosity == PluginIds) {
                cout << "vamp:" << key << endl;
            }
            
            Plugin::OutputList outputs =
                plugin->getOutputDescriptors();

            if (verbosity == PluginInformationDetailed) {

                Plugin::ParameterList params = plugin->getParameterDescriptors();
                for (size_t j = 0; j < params.size(); ++j) {
                    Plugin::ParameterDescriptor &pd(params[j]);
                    cout << "\nParameter " << j+1 << ": \"" << pd.name << "\"" << endl;
                    cout << " - Identifier:         " << pd.identifier << endl;
                    cout << " - Description:        \"" << pd.description << "\"" << endl;
                    if (pd.unit != "") {
                        cout << " - Unit:               " << pd.unit << endl;
                    }
                    cout << " - Range:              ";
                    cout << pd.minValue << " -> " << pd.maxValue << endl;
                    cout << " - Default:            ";
                    cout << pd.defaultValue << endl;
                    if (pd.isQuantized) {
                        cout << " - Quantize Step:      "
                             << pd.quantizeStep << endl;
                    }
                    if (!pd.valueNames.empty()) {
                        cout << " - Value Names:        ";
                        for (size_t k = 0; k < pd.valueNames.size(); ++k) {
                            if (k > 0) cout << ", ";
                            cout << "\"" << pd.valueNames[k] << "\"";
                        }
                        cout << endl;
                    }
                }

                if (outputs.empty()) {
                    cout << "\n** Note: This plugin reports no outputs!" << endl;
                }
                for (size_t j = 0; j < outputs.size(); ++j) {
                    Plugin::OutputDescriptor &od(outputs[j]);
                    cout << "\nOutput " << j+1 << ": \"" << od.name << "\"" << endl;
                    cout << " - Identifier:         " << od.identifier << endl;
                    cout << " - Description:        \"" << od.description << "\"" << endl;
                    if (od.unit != "") {
                        cout << " - Unit:               " << od.unit << endl;
                    }
                    if (od.hasFixedBinCount) {
                        cout << " - Default Bin Count:  " << od.binCount << endl;
                    }
                    if (!od.binNames.empty()) {
                        bool have = false;
                        for (size_t k = 0; k < od.binNames.size(); ++k) {
                            if (od.binNames[k] != "") {
                                have = true; break;
                            }
                        }
                        if (have) {
                            cout << " - Bin Names:          ";
                            for (size_t k = 0; k < od.binNames.size(); ++k) {
                                if (k > 0) cout << ", ";
                                cout << "\"" << od.binNames[k] << "\"";
                            }
                            cout << endl;
                        }
                    }
                    if (od.hasKnownExtents) {
                        cout << " - Default Extents:    ";
                        cout << od.minValue << " -> " << od.maxValue << endl;
                    }
                    if (od.isQuantized) {
                        cout << " - Quantize Step:      "
                             << od.quantizeStep << endl;
                    }
                    cout << " - Sample Type:        "
                         << (od.sampleType ==
                             Plugin::OutputDescriptor::OneSamplePerStep ?
                             "One Sample Per Step" :
                             od.sampleType ==
                             Plugin::OutputDescriptor::FixedSampleRate ?
                             "Fixed Sample Rate" :
                             "Variable Sample Rate") << endl;
                    if (od.sampleType !=
                        Plugin::OutputDescriptor::OneSamplePerStep) {
                        cout << " - Default Rate:       "
                             << od.sampleRate << endl;
                    }
                    cout << " - Has Duration:       "
                         << (od.hasDuration ? "Yes" : "No") << endl;
                }
            }

            if (outputs.size() > 1 || verbosity == PluginOutputIds) {
                for (size_t j = 0; j < outputs.size(); ++j) {
                    if (verbosity == PluginInformation) {
                        cout << "         (" << j << ") "
                             << outputs[j].name << ", \""
                             << outputs[j].identifier << "\"" << endl;
                        if (outputs[j].description != "") {
                            cout << "             - " 
                                 << outputs[j].description << endl;
                        }
                    } else if (verbosity == PluginOutputIds) {
                        cout << "vamp:" << key << ":" << outputs[j].identifier << endl;
                    }
                }
            }

            ++index;

            delete plugin;
        }
    }

    if (verbosity == PluginInformation ||
        verbosity == PluginInformationDetailed) {
        cout << endl;
    }
}

void
printPluginCategoryList()
{
    PluginLoader *loader = PluginLoader::getInstance();

    vector<PluginLoader::PluginKey> plugins = loader->listPlugins();

    set<string> printedcats;

    for (size_t i = 0; i < plugins.size(); ++i) {

        PluginLoader::PluginKey key = plugins[i];
        
        PluginLoader::PluginCategoryHierarchy category =
            loader->getPluginCategory(key);

        Plugin *plugin = loader->loadPlugin(key, 48000);
        if (!plugin) continue;

        string catstr = "";

        if (category.empty()) catstr = '|';
        else {
            for (size_t j = 0; j < category.size(); ++j) {
                catstr += category[j];
                catstr += '|';
                if (printedcats.find(catstr) == printedcats.end()) {
                    std::cout << catstr << std::endl;
                    printedcats.insert(catstr);
                }
            }
        }

        std::cout << catstr << key << ":::" << plugin->getName() << ":::" << plugin->getMaker() << ":::" << plugin->getDescription() << std::endl;
    }
}

