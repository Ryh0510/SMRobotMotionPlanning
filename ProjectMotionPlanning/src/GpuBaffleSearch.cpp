#include "GpuBaffleSearch.h"
#include "GpuBaffleKernel.h"
#include "GpuIkKernel.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

namespace motion_planning { namespace detail {
class GpuBaffleSearch::Impl {
public:
    std::vector<double> triangles,edges;
    std::mutex mutex;
    bool attempted=false,ready=false;
    std::string error,device;
    std::size_t queries=0,taskCount=0,triangleCount=0;
    double elapsedMs=0,initializationMs=0;
    std::size_t ikQueries=0,ikTasks=0,ikConverged=0;double ikMs=0;
#ifdef _WIN32
    using Ptr=unsigned long long;
    HMODULE driver=nullptr,rtc=nullptr,builtins=nullptr;
    void* context=nullptr;void* module=nullptr;void* function=nullptr;void* ikFunction=nullptr;
#define FN(name,signature) using name##_t=int(__stdcall*)signature; name##_t name=nullptr
    FN(cuInit,(unsigned));FN(cuDeviceGet,(int*,int));FN(cuDeviceGetName,(char*,int,int));
    FN(cuDeviceGetAttribute,(int*,int,int));FN(cuCtxCreate_v2,(void**,unsigned,int));
    FN(cuCtxDestroy_v2,(void*));FN(cuCtxPushCurrent_v2,(void*));FN(cuCtxPopCurrent_v2,(void**));
    FN(cuCtxSynchronize,());FN(cuMemAlloc_v2,(Ptr*,std::size_t));FN(cuMemFree_v2,(Ptr));
    FN(cuMemcpyHtoD_v2,(Ptr,const void*,std::size_t));FN(cuMemcpyDtoH_v2,(void*,Ptr,std::size_t));
    FN(cuMemsetD8_v2,(Ptr,unsigned char,std::size_t));
    FN(cuModuleLoadDataEx,(void**,const void*,unsigned,void*,void*));FN(cuModuleUnload,(void*));
    FN(cuModuleGetFunction,(void**,void*,const char*));
    FN(cuLaunchKernel,(void*,unsigned,unsigned,unsigned,unsigned,unsigned,unsigned,unsigned,void*,void**,void**));
    FN(nvrtcCreateProgram,(void**,const char*,const char*,int,const char* const*,const char* const*));
    FN(nvrtcCompileProgram,(void*,int,const char* const*));FN(nvrtcGetPTXSize,(void*,std::size_t*));
    FN(nvrtcGetPTX,(void*,char*));FN(nvrtcGetProgramLogSize,(void*,std::size_t*));
    FN(nvrtcGetProgramLog,(void*,char*));FN(nvrtcDestroyProgram,(void**));
#undef FN
    struct Buffer {Ptr ptr=0;std::size_t capacity=0;};
    Buffer mesh,contours,data,indices,starts,outside,hits;
    Buffer ikModel,ikInputs,ikOutputs;
    template<class T> void symbol(HMODULE library,T& destination,const char* name){
        destination=reinterpret_cast<T>(GetProcAddress(library,name));
        if(!destination)throw std::runtime_error(std::string("CUDA runtime missing symbol: ")+name);
    }
    void check(int code,const char* operation){if(code)throw std::runtime_error(std::string(operation)+" failed (CUDA code "+std::to_string(code)+")");}
    void reserve(Buffer& b,std::size_t bytes){
        if(b.capacity>=bytes)return;
        if(b.ptr)check(cuMemFree_v2(b.ptr),"cuMemFree");b={};
        check(cuMemAlloc_v2(&b.ptr,std::max<std::size_t>(bytes,8)),"cuMemAlloc");b.capacity=std::max<std::size_t>(bytes,8);
    }
    void upload(Buffer& b,const void* source,std::size_t bytes){reserve(b,bytes);if(bytes)check(cuMemcpyHtoD_v2(b.ptr,source,bytes),"cuMemcpyHtoD");}
    std::filesystem::path runtimeDirectory(){
        wchar_t exe[32768];const auto length=GetModuleFileNameW(nullptr,exe,32768);
        const auto local=std::filesystem::path(std::wstring(exe,length)).parent_path()/L"cuda";
        wchar_t configured[32768];const auto size=GetEnvironmentVariableW(L"RWS_CUDA_RUNTIME",configured,32768);
        return size>0 && size<32768?std::filesystem::path(std::wstring(configured,size)):local;
    }
    void initialize(){
        if(attempted){if(!ready)throw std::runtime_error(error);return;}
        attempted=true;const auto began=std::chrono::steady_clock::now();
        try {
            driver=LoadLibraryExW(L"nvcuda.dll",nullptr,LOAD_LIBRARY_SEARCH_SYSTEM32);
            if(!driver)throw std::runtime_error("NVIDIA CUDA driver unavailable");
#define LOAD(name) symbol(driver,name,#name)
            LOAD(cuInit);LOAD(cuDeviceGet);LOAD(cuDeviceGetName);LOAD(cuDeviceGetAttribute);
            LOAD(cuCtxCreate_v2);LOAD(cuCtxDestroy_v2);LOAD(cuCtxPushCurrent_v2);LOAD(cuCtxPopCurrent_v2);LOAD(cuCtxSynchronize);
            LOAD(cuMemAlloc_v2);LOAD(cuMemFree_v2);LOAD(cuMemcpyHtoD_v2);LOAD(cuMemcpyDtoH_v2);LOAD(cuMemsetD8_v2);
            LOAD(cuModuleLoadDataEx);LOAD(cuModuleUnload);LOAD(cuModuleGetFunction);LOAD(cuLaunchKernel);
#undef LOAD
            const auto directory=runtimeDirectory();std::filesystem::path compiler,builtin;
            if(std::filesystem::exists(directory))for(const auto& entry:std::filesystem::directory_iterator(directory)){
                const auto name=entry.path().filename().wstring();
                if(entry.path().extension()!=L".dll")continue;
                if(name.rfind(L"nvrtc-builtins",0)==0)builtin=entry.path();
                else if(name.rfind(L"nvrtc64_",0)==0 && name.find(L".alt.")==std::wstring::npos)compiler=entry.path();
            }
            if(!builtin.empty())builtins=LoadLibraryExW(builtin.c_str(),nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
            if(!compiler.empty())rtc=LoadLibraryExW(compiler.c_str(),nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
            if(!rtc)throw std::runtime_error("NVRTC unavailable; run tools/install_gpu_runtime.ps1");
#define LOAD(name) symbol(rtc,name,#name)
            LOAD(nvrtcCreateProgram);LOAD(nvrtcCompileProgram);LOAD(nvrtcGetPTXSize);LOAD(nvrtcGetPTX);
            LOAD(nvrtcGetProgramLogSize);LOAD(nvrtcGetProgramLog);LOAD(nvrtcDestroyProgram);
#undef LOAD
            check(cuInit(0),"cuInit");int gpu=0;check(cuDeviceGet(&gpu,0),"cuDeviceGet");
            char label[256]={};check(cuDeviceGetName(label,256,gpu),"cuDeviceGetName");device=label;
            int major=0,minor=0;check(cuDeviceGetAttribute(&major,75,gpu),"compute capability");check(cuDeviceGetAttribute(&minor,76,gpu),"compute capability");
            const auto source=std::string(gpuBaffleKernel)+gpuIkKernel;
            void* program=nullptr;check(nvrtcCreateProgram(&program,source.c_str(),"rws_planning.cu",0,nullptr,nullptr),"nvrtcCreateProgram");
            const std::string architecture="--gpu-architecture=compute_"+std::to_string(major)+std::to_string(minor);
            const char* options[]={architecture.c_str(),"--std=c++14","--fmad=false"};
            // NVRTC uses a deep compiler stack. Keep it off the caller's Debug
            // planning/Eigen stack; source, options and CUDA ownership are unchanged.
            int compiled=0;
            try {
                std::thread compilerThread([&](){compiled=nvrtcCompileProgram(program,3,options);});
                compilerThread.join();
            } catch(...) {nvrtcDestroyProgram(&program);throw;}
            if(compiled){std::size_t count=0;nvrtcGetProgramLogSize(program,&count);std::string log(count,'\0');if(count)nvrtcGetProgramLog(program,log.data());nvrtcDestroyProgram(&program);throw std::runtime_error("NVRTC compile: "+log.substr(0,4000));}
            std::size_t count=0;check(nvrtcGetPTXSize(program,&count),"nvrtcGetPTXSize");std::vector<char> ptx(count);check(nvrtcGetPTX(program,ptx.data()),"nvrtcGetPTX");nvrtcDestroyProgram(&program);
            check(cuCtxCreate_v2(&context,0,gpu),"cuCtxCreate");
            try {check(cuModuleLoadDataEx(&module,ptx.data(),0,nullptr,nullptr),"cuModuleLoadData");check(cuModuleGetFunction(&function,module,"scoreMesh"),"cuModuleGetFunction");check(cuModuleGetFunction(&ikFunction,module,"solveSerialIk"),"cuModuleGetFunction IK");
                upload(mesh,triangles.data(),triangles.size()*sizeof(double));upload(contours,edges.data(),edges.size()*sizeof(double));
            }catch(...){void* popped=nullptr;cuCtxPopCurrent_v2(&popped);throw;}
            void* popped=nullptr;check(cuCtxPopCurrent_v2(&popped),"cuCtxPopCurrent");ready=true;
            initializationMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
        }catch(const std::exception& e){error=e.what();ready=false;throw;}
    }
    ~Impl(){
        if(context && cuCtxPushCurrent_v2 && cuCtxPushCurrent_v2(context)==0){
            for(auto* b:{&mesh,&contours,&data,&indices,&starts,&outside,&hits,&ikModel,&ikInputs,&ikOutputs})if(b->ptr)cuMemFree_v2(b->ptr);
            if(module)cuModuleUnload(module);void* popped=nullptr;cuCtxPopCurrent_v2(&popped);cuCtxDestroy_v2(context);
        }
        if(rtc)FreeLibrary(rtc);if(builtins)FreeLibrary(builtins);if(driver)FreeLibrary(driver);
    }
#endif
};
GpuBaffleSearch::GpuBaffleSearch():impl(new Impl){}
GpuBaffleSearch::~GpuBaffleSearch()=default;
void GpuBaffleSearch::configure(std::vector<double> triangles,std::vector<double> edges){
    std::lock_guard<std::mutex> guard(impl->mutex);
    if(impl->attempted)throw std::runtime_error("Cannot reconfigure active GPU scene");
    if(triangles.size()%9 || edges.size()%4 ||
        !std::all_of(triangles.begin(),triangles.end(),[](double x){return std::isfinite(x);}) ||
        !std::all_of(edges.begin(),edges.end(),[](double x){return std::isfinite(x);}))throw std::runtime_error("Invalid CUDA geometry layout");
    impl->triangles=std::move(triangles);impl->edges=std::move(edges);
}
bool GpuBaffleSearch::evaluate(const std::vector<GpuBaffleTask>& tasks,std::vector<double>& scores){
    std::lock_guard<std::mutex> guard(impl->mutex);scores.clear();
#ifdef _WIN32
    bool pushed=false;
    try {
        if(tasks.empty())return true;
        if(tasks.size()>4096)throw std::runtime_error("GPU task batch exceeds bound");
        std::vector<double> geometry;std::vector<int> offsets(6*tasks.size());std::vector<unsigned long long> begins{0};
        std::size_t owner=0;
        for(const auto& t:tasks){
            if(t.triangleBegin<0 || t.triangleCount<=0 || std::size_t(t.triangleBegin)+t.triangleCount>impl->triangles.size()/9 ||
                t.edgeBegin<0 || t.edgeCount<=0 || std::size_t(t.edgeBegin)+t.edgeCount>impl->edges.size()/4 ||
                !std::all_of(t.geometry.begin(),t.geometry.end(),[](double x){return std::isfinite(x);}) || t.geometry[24]<0 || t.geometry[25]<0)
                throw std::runtime_error("Invalid CUDA task geometry");
            geometry.insert(geometry.end(),t.geometry.begin(),t.geometry.end());
            const auto metadata=6*owner++;
            offsets[metadata]=t.triangleBegin;offsets[metadata+1]=t.triangleCount;offsets[metadata+2]=t.edgeBegin;offsets[metadata+3]=t.edgeCount;
            offsets[metadata+4]=int(offsets.size());offsets[metadata+5]=t.triangleIndices.empty()?0:1;
            for(int index:t.triangleIndices){if(index<0 || index>=t.triangleCount)throw std::runtime_error("Invalid sparse GPU triangle index");offsets.push_back(t.triangleBegin+index);}
            begins.push_back(begins.back()+(t.triangleIndices.empty()?std::size_t(t.triangleCount):t.triangleIndices.size()));
        }
        impl->initialize();const auto began=std::chrono::steady_clock::now();
        impl->check(impl->cuCtxPushCurrent_v2(impl->context),"cuCtxPushCurrent");pushed=true;
        impl->upload(impl->data,geometry.data(),geometry.size()*sizeof(double));impl->upload(impl->indices,offsets.data(),offsets.size()*sizeof(int));
        impl->upload(impl->starts,begins.data(),begins.size()*sizeof(unsigned long long));
        impl->reserve(impl->outside,tasks.size()*sizeof(double));impl->reserve(impl->hits,tasks.size()*sizeof(unsigned));
        impl->check(impl->cuMemsetD8_v2(impl->outside.ptr,0,tasks.size()*sizeof(double)),"clear merits");
        impl->check(impl->cuMemsetD8_v2(impl->hits.ptr,0,tasks.size()*sizeof(unsigned)),"clear hits");
        int count=int(tasks.size());
        // Bounded kernels avoid long display-GPU watchdog stalls on large meshes.
        for(unsigned long long base=0;base<begins.back();base+=1048576){
            unsigned long long end=std::min(begins.back(),base+1048576);
            void* args[]={&impl->mesh.ptr,&impl->contours.ptr,&impl->data.ptr,&impl->indices.ptr,&impl->starts.ptr,&count,&base,&end,&impl->outside.ptr,&impl->hits.ptr};
            impl->check(impl->cuLaunchKernel(impl->function,unsigned((end-base+255)/256),1,1,256,1,1,0,nullptr,args,nullptr),"scoreMesh launch");
        }
        impl->check(impl->cuCtxSynchronize(),"scoreMesh synchronize");
        std::vector<double> outside(tasks.size());std::vector<unsigned> hits(tasks.size());
        impl->check(impl->cuMemcpyDtoH_v2(outside.data(),impl->outside.ptr,outside.size()*sizeof(double)),"read merits");
        impl->check(impl->cuMemcpyDtoH_v2(hits.data(),impl->hits.ptr,hits.size()*sizeof(unsigned)),"read hits");
        scores.resize(tasks.size());for(std::size_t i=0;i<tasks.size();++i){scores[i]=outside[i]+.001*hits[i]/tasks[i].triangleCount;if(!std::isfinite(scores[i]) || hits[i]>unsigned(tasks[i].triangleCount))throw std::runtime_error("Invalid CUDA merit output");}
        void* popped=nullptr;impl->check(impl->cuCtxPopCurrent_v2(&popped),"cuCtxPopCurrent");pushed=false;
        impl->queries++;impl->taskCount+=tasks.size();impl->triangleCount+=begins.back();
        impl->elapsedMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();return true;
    }catch(const std::exception& e){
        if(pushed){void* popped=nullptr;impl->cuCtxPopCurrent_v2(&popped);}
        impl->error=e.what();impl->ready=false;scores.clear();return false;
    }
#else
    impl->attempted=true;impl->error="CUDA dynamic backend is available on Windows only";return false;
#endif
}
nlohmann::json GpuBaffleSearch::status() const {
    std::lock_guard<std::mutex> guard(impl->mutex);
    return {{"attempted",impl->attempted},{"available",impl->ready},{"device",impl->device},{"error",impl->error},
        {"batch_queries",impl->queries},{"body_tasks",impl->taskCount},{"triangles_scored",impl->triangleCount},
        {"query_ms",impl->elapsedMs},{"initialization_ms",impl->initializationMs},
        {"ik_batches",impl->ikQueries},{"ik_tasks",impl->ikTasks},{"ik_converged",impl->ikConverged},{"ik_ms",impl->ikMs},
        {"scope","actual-model IK candidates and mesh/slab/aperture merit; independent CPU FK and safety gates required"}};
}
bool GpuBaffleSearch::solveIk(const GpuSerialIkModel& model,const std::vector<GpuIkTask>& tasks,
    std::vector<GpuIkRoot>& roots,int iterations,double positionTolerance,double orientationTolerance,double damping,double step){
    std::lock_guard<std::mutex> guard(impl->mutex);roots.clear();
#ifdef _WIN32
    bool pushed=false;
    try {
        if(tasks.empty())return true;
        if(tasks.size()>4096 || iterations<1 || iterations>1000 || !std::isfinite(positionTolerance) || positionTolerance<=0 ||
            !std::isfinite(orientationTolerance) || orientationTolerance<=0 || !std::isfinite(damping) || damping<=0 || !std::isfinite(step) || step<=0 || step>1 ||
            !std::all_of(model.begin(),model.end(),[](double x){return std::isfinite(x);}))throw std::runtime_error("Invalid GPU IK batch/options");
        std::vector<double> input;input.reserve(tasks.size()*18);
        for(const auto& t:tasks){for(double x:t.target)if(!std::isfinite(x))throw std::runtime_error("Non-finite GPU IK target");for(double x:t.seed)if(!std::isfinite(x))throw std::runtime_error("Non-finite GPU IK seed");input.insert(input.end(),t.target.begin(),t.target.end());input.insert(input.end(),t.seed.begin(),t.seed.end());}
        impl->initialize();const auto began=std::chrono::steady_clock::now();
        impl->check(impl->cuCtxPushCurrent_v2(impl->context),"cuCtxPushCurrent IK");pushed=true;
        impl->upload(impl->ikModel,model.data(),model.size()*sizeof(double));impl->upload(impl->ikInputs,input.data(),input.size()*sizeof(double));impl->reserve(impl->ikOutputs,tasks.size()*7*sizeof(double));
        int count=int(tasks.size());
        void* args[]={&impl->ikModel.ptr,&impl->ikInputs.ptr,&count,&iterations,&positionTolerance,&orientationTolerance,&damping,&step,&impl->ikOutputs.ptr};
        impl->check(impl->cuLaunchKernel(impl->ikFunction,unsigned((count+63)/64),1,1,64,1,1,0,nullptr,args,nullptr),"solveSerialIk launch");
        impl->check(impl->cuCtxSynchronize(),"solveSerialIk synchronize");std::vector<double> output(tasks.size()*7);
        impl->check(impl->cuMemcpyDtoH_v2(output.data(),impl->ikOutputs.ptr,output.size()*sizeof(double)),"read IK roots");
        roots.resize(tasks.size());for(std::size_t i=0;i<tasks.size();++i){auto& r=roots[i];r.converged=output[7*i+6]==1.;for(int j=0;j<6;++j){r.joints[j]=output[7*i+j];if(!std::isfinite(r.joints[j]))r.converged=false;}if(r.converged)++impl->ikConverged;}
        void* popped=nullptr;impl->check(impl->cuCtxPopCurrent_v2(&popped),"cuCtxPopCurrent IK");pushed=false;
        ++impl->ikQueries;impl->ikTasks+=tasks.size();impl->ikMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();return true;
    }catch(const std::exception& e){if(pushed){void* popped=nullptr;impl->cuCtxPopCurrent_v2(&popped);}impl->error=e.what();impl->ready=false;roots.clear();return false;}
#else
    impl->attempted=true;impl->error="CUDA dynamic backend is available on Windows only";return false;
#endif
}
}}
