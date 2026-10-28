#include <PostModel/PostInterface.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <regex>
#include <vector>
#include <string>
#include <algorithm>
#include <windows.h>
#include <tlhelp32.h>
#include <psapi.h>
#include <locale>
#include <codecvt>
#pragma execution_character_set("utf-8")

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dlfcn.h>
#endif

std::string getCurrentModulePath() {
    char path[MAX_PATH];
    HMODULE hm = NULL;
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, (LPCSTR)&getCurrentModulePath, &hm)) {
        GetModuleFileNameA(hm, path, sizeof(path));
        std::string fullPath(path);
        size_t pos = fullPath.find_last_of("\\/");
        return fullPath.substr(0, pos + 1); // 返回目录路径
    }
    return "";
}

// 带自动注册功能的构造函数
PostInterface::PostInterface() {
    std::string dllDirectory = getCurrentModulePath();
    std::cout << "PostInterface constructor with auto-registration from directory: " << dllDirectory << std::endl;

    // 检查目录是否存在
#ifdef _WIN32
    DWORD attributes = GetFileAttributesA(dllDirectory.c_str());
    if (attributes == INVALID_FILE_ATTRIBUTES) {
        std::cerr << "Warning: Directory does not exist: " << dllDirectory << std::endl;
        return;
    }

    if (!(attributes & FILE_ATTRIBUTE_DIRECTORY)) {
        std::cerr << "Warning: Path is not a directory: " << dllDirectory << std::endl;
        return;
    }
#else
    struct stat info;
    if (stat(dllDirectory.c_str(), &info) != 0) {
        std::cerr << "Warning: Directory does not exist: " << dllDirectory << std::endl;
        return;
    }

    if (!S_ISDIR(info.st_mode)) {
        std::cerr << "Warning: Path is not a directory: " << dllDirectory << std::endl;
        return;
    }
#endif

    // 扫描目录中的DLL文件并自动注册
    autoRegisterFromDLLs(dllDirectory);
}

PostInterface& PostInterface::getInstance() {
    static PostInterface instance;
    return instance;
}

void PostInterface::registerTemplate(const std::string& name,
    const std::string& headerPath,
    const std::string& cppPath,
    const std::string& className,
    const std::string& headerContent,
    const std::string& cppContent) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    std::cout << "=== 注册模板调试信息 ===" << std::endl;
    std::cout << "名称: " << name << std::endl;
    std::cout << "头文件路径: " << headerPath << std::endl;
    std::cout << "源文件路径: " << cppPath << std::endl;
    std::cout << "类名: " << className << std::endl;
    std::cout << "头文件内容大小: " << headerContent.size() << " 字节" << std::endl;
    std::cout << "源文件内容大小: " << cppContent.size() << " 字节" << std::endl;
     
    const std::string headerpath = __FILE__;
    // 安全的路径计算
    std::string ftpheaderpath = headerpath;
    size_t slashCount = 0;
    size_t lastSlash = ftpheaderpath.find_last_of("/\\");

    while (lastSlash != std::string::npos && slashCount < 3) {
        ftpheaderpath = ftpheaderpath.substr(0, lastSlash);
        lastSlash = ftpheaderpath.find_last_of("/\\");
        slashCount++;
    }

    if (slashCount < 3) {
        std::cerr << "ERROR: Cannot calculate proper include path from: " << headerpath << std::endl;
        return;
    }
    std::string mathheaderpath = ftpheaderpath;
    ftpheaderpath += "/PostModel/include/PostModel/ftpsend.h";
    mathheaderpath += "/PostModel/include/PostModel/robomath.h";

    const std::string mathcpppath = headerpath.substr(0, headerpath.find_last_of("/\\")) + "/robomath.cpp";
    const std::string ftpcpppath = headerpath.substr(0, headerpath.find_last_of("/\\")) + "/ftpsend.cpp";

    TemplateInfo info;
    info.headerPath = headerPath;
    info.cppPath = cppPath;
    info.className = className;
    info.templateName = name;
    info.headerContent = headerContent;  // 新增字段
    info.cppContent = cppContent;       // 新增字段
    info.sourceEmbedded = !headerContent.empty() && !cppContent.empty();         // 标记源文件已嵌入

    std::stringstream buffer;
    std::ifstream file(mathheaderpath);
    file.is_open();
    buffer << file.rdbuf();
    file.close();
    info.mathheader = buffer.str();
    buffer.clear();

    std::stringstream bufferCpp;
    std::ifstream file1(mathcpppath);
    file1.is_open();
    bufferCpp << file1.rdbuf();
    file1.close();
    info.mathcppContent = bufferCpp.str();
    bufferCpp.clear();

    std::stringstream ftpbuffer;
    std::ifstream file2(ftpheaderpath);
    file2.is_open();
    ftpbuffer << file2.rdbuf();
    file2.close();
    info.ftpheader = ftpbuffer.str();
    ftpbuffer.clear();

    std::stringstream ftpbufferCpp;
    std::ifstream file3(ftpcpppath);
    file3.is_open();
    ftpbufferCpp << file3.rdbuf();
    file3.close();
    info.ftpcppContent = ftpbufferCpp.str();
    ftpbufferCpp.clear();

    templates_[name] = info;
}

void PostInterface::unregisterTemplate(const std::string& name) {
    auto it = templates_.find(name);
    if (it != templates_.end()) {
        // 如果从DLL加载，需要卸载
        if (it->second.dllHandle) {
#ifdef _WIN32
            FreeLibrary((HMODULE)it->second.dllHandle);
#else
            dlclose(it->second.dllHandle);
#endif
        }
        templates_.erase(it);
        std::cout << "Unregistered template: " << name << std::endl;
    }
}

std::vector<std::string> PostInterface::getAvailableTemplates() const {
    std::vector<std::string> result;
    for (const auto& pair : templates_) {
        result.push_back(pair.first);
    }
    return result;
}

TemplateInfo PostInterface::getTemplateInfo(const std::string& templateName) const {
    auto it = templates_.find(templateName);
    if (it == templates_.end()) {
        throw std::runtime_error("Template not found: " + templateName);
    }
    return it->second;
}

// 修改：从DLL加载模板，使用模板特定的注册函数名
bool PostInterface::loadTemplateFromDLL(const std::string& dllPath,
    const std::string& templateName) {

    // 构建模板特定的注册函数名
    std::string registerFunctionName = "register_" + templateName;

#ifdef _WIN32
    HMODULE dllHandle = LoadLibraryA(dllPath.c_str());
    if (!dllHandle) {
        std::cerr << "Cannot load DLL: " << dllPath << std::endl;
        return false;
    }

    // 查找模板特定的注册函数
    auto registerFunc = (void(*)(PostInterface&))GetProcAddress(dllHandle, registerFunctionName.c_str());
    if (!registerFunc) {
        std::cerr << "Cannot find register function '" << registerFunctionName << "' in DLL" << std::endl;
        FreeLibrary(dllHandle);
        return false;
    }

    // 调用注册函数
    registerFunc(*this);

    // 保存DLL句柄
    templates_[templateName].dllHandle = dllHandle;
#else
    void* dllHandle = dlopen(dllPath.c_str(), RTLD_LAZY);
    if (!dllHandle) {
        std::cerr << "Cannot load DLL: " << dllPath << " Error: " << dlerror() << std::endl;
        return false;
    }

    auto registerFunc = (void(*)(PostInterface&))dlsym(dllHandle, registerFunctionName.c_str());
    if (!registerFunc) {
        std::cerr << "Cannot find register function '" << registerFunctionName << "' in DLL" << std::endl;
        dlclose(dllHandle);
        return false;
    }

    registerFunc(*this);
    templates_[templateName].dllHandle = dllHandle;
#endif

    std::cout << "Template loaded from DLL: " << templateName << " using function: " << registerFunctionName << std::endl;
    return true;
}

#ifdef _WIN32
std::vector<std::string> PostInterface::listFilesInDirectory(const std::string& directory, const std::string& extension) {
    std::vector<std::string> files;

    WIN32_FIND_DATAA findData;
    HANDLE hFind;
    std::string searchPath = directory + "\\*" + extension;

    hFind = FindFirstFileA(searchPath.c_str(), &findData);
    if (hFind == INVALID_HANDLE_VALUE) {
        return files;
    }

    do {
        if (!(findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
            files.push_back(findData.cFileName);
        }
    } while (FindNextFileA(hFind, &findData) != 0);

    FindClose(hFind);
    return files;
}

bool PostInterface::isRegularFile(const std::string& path) {
    DWORD attributes = GetFileAttributesA(path.c_str());
    return (attributes != INVALID_FILE_ATTRIBUTES &&
        !(attributes & FILE_ATTRIBUTE_DIRECTORY));
}

#else
// Linux/Unix 版本的文件列表获取
std::vector<std::string> PostInterface::listFilesInDirectory(const std::string& directory, const std::string& extension) {
    std::vector<std::string> files;

    DIR* dir = opendir(directory.c_str());
    if (!dir) {
        return files;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;

        // 跳过 . 和 .. 目录
        if (filename == "." || filename == "..") {
            continue;
        }

        // 检查文件扩展名
        if (filename.length() > extension.length() &&
            filename.substr(filename.length() - extension.length()) == extension) {
            files.push_back(filename);
        }
    }

    closedir(dir);
    return files;
}

bool PostInterface::isRegularFile(const std::string& path) {
    struct stat pathStat;
    if (stat(path.c_str(), &pathStat) != 0) {
        return false;
    }
    return S_ISREG(pathStat.st_mode);
}
#endif

// 修改：提取模板特定的注册函数名
std::string PostInterface::extractTemplateRegisterFunction(const std::string& content, const std::string& templateName) {
    // 查找模板特定的注册函数模式
    std::regex pattern("extern\\s+\"C\"\\s+__declspec\\(dllexport\\)\\s+inline\\s+void\\s+(register_[A-Za-z0-9_]+)");
    std::smatch match;

    if (std::regex_search(content, match, pattern)) {
        if (match.size() > 1) {
            return match[1].str();
        }
    }

    // 如果没有找到特定函数，尝试构建默认名称
    return "register_" + templateName;
}

void PostInterface::autoRegisterTemplates(const std::string& templatesDirectory) {
    std::cout << "Scanning for templates in: " << templatesDirectory << std::endl;

    // 获取所有 .h 和 .cpp 文件
    std::vector<std::string> headerFiles = listFilesInDirectory(templatesDirectory, ".h");
    std::vector<std::string> cppFiles = listFilesInDirectory(templatesDirectory, ".cpp");

    if (headerFiles.empty()) {
        std::cout << "No header files found in directory." << std::endl;
        return;
    }

    std::cout << "Found " << headerFiles.size() << " header files and "
        << cppFiles.size() << " cpp files." << std::endl;

    int registeredCount = 0;

    for (const auto& filename : headerFiles) {
        // 构建完整路径
        std::string headerPath = templatesDirectory;
#ifdef _WIN32
        headerPath += "\\" + filename;
#else
        headerPath += "/" + filename;
#endif

        // 检查是否为常规文件
        if (!isRegularFile(headerPath)) {
            continue;
        }

        // 从文件名提取模板名称
        std::string templateName = filename;
        size_t dotPos = templateName.find_last_of('.');
        if (dotPos != std::string::npos) {
            templateName = templateName.substr(0, dotPos);
        }

        // 查找对应的cpp文件
        std::string cppFilename = templateName + ".cpp";
        std::string cppPath = templatesDirectory;
#ifdef _WIN32
        cppPath += "\\" + cppFilename;
#else
        cppPath += "/" + cppFilename;
#endif

        if(!isRegularFile(cppPath)) {
            const size_t lastSlash = templatesDirectory.find_last_of("\\/");
            if(lastSlash != std::string::npos) {
                const std::string includeRoot = templatesDirectory.substr(0, lastSlash);
                const size_t parentSlash = includeRoot.find_last_of("\\/");
                if(parentSlash != std::string::npos) {
                    const std::string packageRoot = includeRoot.substr(0, parentSlash);
#ifdef _WIN32
                    cppPath = packageRoot + "\\src\\" + cppFilename;
#else
                    cppPath = packageRoot + "/src/" + cppFilename;
#endif
                }
            }
        }

        // 检查cpp文件是否存在
        const bool cppExists = isRegularFile(cppPath);

        if (!cppExists) {
            std::cout << "Warning: No corresponding CPP file for: " << templateName << std::endl;
            continue;
        }

        // 检查文件内容
        std::ifstream file(headerPath);
        if (!file.is_open()) {
            std::cout << "Cannot open file: " << headerPath << std::endl;
            continue;
        }

        std::string content((std::istreambuf_iterator<char>(file)),
            std::istreambuf_iterator<char>());
        file.close();

        // 修改：检查是否包含模板特定的注册函数
        std::string registerFunction = extractTemplateRegisterFunction(content, templateName);
        if (content.find(registerFunction) != std::string::npos) {
            // 提取类名
            std::string className = extractClassNameFromFunction(content, "RobotPost");

            // 需要读取头文件和cpp文件的内容
            std::string headerContent, cppContent;

            // 读取头文件内容
            std::ifstream headerFile(headerPath);
            if (headerFile.is_open()) {
                headerContent = std::string((std::istreambuf_iterator<char>(headerFile)),
                    std::istreambuf_iterator<char>());
                headerFile.close();
            }

            // 读取cpp文件内容
            std::ifstream cppFile(cppPath);
            if (cppFile.is_open()) {
                cppContent = std::string((std::istreambuf_iterator<char>(cppFile)),
                    std::istreambuf_iterator<char>());
                cppFile.close();
            }

            registerTemplate(templateName, headerPath, cppPath, className, headerContent, cppContent);
            std::cout << "✓ Auto-registered template: " << templateName
                << " (Class: " << className << ", Register Function: " << registerFunction << ")" << std::endl;
            registeredCount++;
        }
    }

    std::cout << "Scan completed. Registered " << registeredCount << " templates." << std::endl;
}

// 从注册函数中提取类名
std::string PostInterface::extractClassNameFromFunction(const std::string& content, const std::string& defaultName) {
    // 查找注册函数模式
    std::regex pattern("manager\\.registerTemplate\\(\"[^\"]+\",\\s*\"[^\"]+\",\\s*\"[^\"]+\",\\s*\"([^\"]+)\"\\)");
    std::smatch match;

    if (std::regex_search(content, match, pattern) && match.size() > 1) {
        return match[1].str();
    }

    return defaultName;
}

bool PostInterface::autoRegisterFromDLL(const std::string& dllPath) {
#ifdef _WIN32
    HMODULE dllHandle = LoadLibraryA(dllPath.c_str());
    if (!dllHandle) {
        std::cerr << "Cannot load DLL: " << dllPath << std::endl;
        return false;
    }

    std::cout << "Scanning DLL for templates: " << dllPath << std::endl;

    // 获取导出表来查找所有以"register_"开头的函数
    BYTE* const moduleBase = reinterpret_cast<BYTE*>(dllHandle);
    IMAGE_DOS_HEADER* dosHeader = reinterpret_cast<IMAGE_DOS_HEADER*>(moduleBase);
    if (dosHeader->e_magic != IMAGE_DOS_SIGNATURE) {
        std::cerr << "Invalid DOS header: " << dllPath << std::endl;
        FreeLibrary(dllHandle);
        return false;
    }

    IMAGE_NT_HEADERS* ntHeader = reinterpret_cast<IMAGE_NT_HEADERS*>(moduleBase + dosHeader->e_lfanew);
    if (ntHeader->Signature != IMAGE_NT_SIGNATURE) {
        std::cerr << "Invalid NT header: " << dllPath << std::endl;
        FreeLibrary(dllHandle);
        return false;
    }

    const IMAGE_DATA_DIRECTORY& exportDataDirectory =
        ntHeader->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_EXPORT];
    if (exportDataDirectory.VirtualAddress == 0 || exportDataDirectory.Size == 0) {
        std::cerr << "DLL has no export directory: " << dllPath << std::endl;
        FreeLibrary(dllHandle);
        return false;
    }

    IMAGE_EXPORT_DIRECTORY* exportDir = reinterpret_cast<IMAGE_EXPORT_DIRECTORY*>(
        moduleBase + exportDataDirectory.VirtualAddress);
    if (exportDir->AddressOfNames == 0 || exportDir->NumberOfNames == 0) {
        std::cerr << "DLL has no exported names: " << dllPath << std::endl;
        FreeLibrary(dllHandle);
        return false;
    }

    DWORD* names = reinterpret_cast<DWORD*>(moduleBase + exportDir->AddressOfNames);

    int registeredCount = 0;

    for (DWORD i = 0; i < exportDir->NumberOfNames; i++) {
        if (names[i] == 0) {
            continue;
        }

        char* functionName = reinterpret_cast<char*>(moduleBase + names[i]);
        if (functionName == nullptr || functionName[0] == '\0') {
            continue;
        }
        std::string funcNameStr = functionName;

        // 检查是否是模板注册函数（以register_开头）
        if (funcNameStr.find("register_") == 0) {
            // 提取模板名称（去掉"register_"前缀）
            std::string templateName = funcNameStr.substr(9);

            // 检查是否已经注册
            if (templates_.find(templateName) != templates_.end()) {
                std::cout << "Template already registered: " << templateName << std::endl;
                continue;
            }

            std::cout << "Found template: " << templateName << " (function: " << funcNameStr << ")" << std::endl;

            // 调用注册函数
            auto registerFunc = (void(*)(PostInterface&))GetProcAddress(dllHandle, functionName);
            if (registerFunc) {
                registerFunc(*this);
                templates_[templateName].dllHandle = dllHandle;
                std::cout << "Registered template: " << templateName << std::endl;
                registeredCount++;
            }
        }
    }

    if (registeredCount > 0) {
        std::cout << "Successfully registered " << registeredCount << " templates from DLL" << std::endl;
        return true;
    }
    else {
        std::cerr << "No template registration functions found in DLL" << std::endl;
        FreeLibrary(dllHandle);
        return false;
    }
#else
    // Linux版本类似逻辑
    return false;
#endif
}

bool PostInterface::autoRegisterFromDLLs(const std::string& dllDirectory) {
    std::cout << "Scanning for template DLLs in: " << dllDirectory << std::endl;

    std::vector<std::string> dllFiles;
#ifdef _WIN32
    dllFiles = listFilesInDirectory(dllDirectory, ".dll");
#else
    dllFiles = listFilesInDirectory(dllDirectory, ".so");
#endif

    if (dllFiles.empty()) {
        std::cout << "No DLL files found. Trying to compile templates first..." << std::endl;
        // 先尝试从源代码注册模板
        autoRegisterTemplates(dllDirectory);
        return false;
    }

    std::cout << "Found " << dllFiles.size() << " DLL files." << std::endl;

    int loadedCount = 0;
    for (const auto& filename : dllFiles) {
        if (filename.find("PostModel") == std::string::npos &&
            filename.find("postmodel") == std::string::npos) {
            continue;
        }

        std::string fullPath = dllDirectory;
#ifdef _WIN32
        fullPath += "\\" + filename;
#else
        fullPath += "/" + filename;
#endif

        // 从文件名提取模板名称
        std::string templateName = filename;
        size_t lastDot = templateName.find_last_of('.');
        if (lastDot != std::string::npos) {
            templateName = templateName.substr(0, lastDot);
        }

        if (autoRegisterFromDLL(fullPath)) {
            loadedCount++;
        }
    }

    std::cout << "DLL scan completed. Loaded " << loadedCount << " templates." << std::endl;
    return loadedCount > 0;
}

bool PostInterface::findTemplateFiles(const std::string& templateName,
    std::string& headerPath,
    std::string& cppPath) const {
    auto it = templates_.find(templateName);
    if (it == templates_.end()) {
        return false;
    }

    headerPath = it->second.headerPath;
    cppPath = it->second.cppPath;
    return true;
}

bool PostInterface::copyTemplate(const std::string& sourceTemplateName,
    const std::string& newTemplateName,
    const std::string& newHeaderPath,
    const std::string& newCppPath) {
    // 查找源模板
    auto it = templates_.find(sourceTemplateName);
    if (it == templates_.end()) {
        std::cerr << "Source template not found: " << sourceTemplateName << std::endl;
        return false;
    }

    // 复制头文件
    std::ifstream srcHeader(it->second.headerPath, std::ios::binary);
    std::ofstream dstHeader(newHeaderPath, std::ios::binary);
    if (!srcHeader.is_open() || !dstHeader.is_open()) {
        std::cerr << "Cannot copy header file" << std::endl;
        return false;
    }
    dstHeader << srcHeader.rdbuf();
    srcHeader.close();
    dstHeader.close();

    // 复制cpp文件
    std::ifstream srcCpp(it->second.cppPath, std::ios::binary);
    std::ofstream dstCpp(newCppPath, std::ios::binary);
    if (!srcCpp.is_open() || !dstCpp.is_open()) {
        std::cerr << "Cannot copy cpp file" << std::endl;
        return false;
    }
    dstCpp << srcCpp.rdbuf();
    srcCpp.close();
    dstCpp.close();

    std::cout << "Template copied successfully: " << newTemplateName << std::endl;
    return true;
}

PostInterface::~PostInterface() {
    // 清理所有DLL句柄
    for (auto& pair : templates_) {
        if (pair.second.dllHandle) {
#ifdef _WIN32
            FreeLibrary((HMODULE)pair.second.dllHandle);
#else
            dlclose(pair.second.dllHandle);
#endif
        }
    }
}


