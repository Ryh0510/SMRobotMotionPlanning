#pragma once
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <memory>
#include "PostInterface-global.h"

class RobotPost;

// 模板文件信息结构
struct POSTINTERFACE_SHAREDLIB TemplateInfo {
    std::string headerPath;
    std::string cppPath;
    std::string className;
    void* dllHandle = nullptr;
    std::string templateName; // 添加模板名称字段

    // 新增：源文件内容
   std::string headerContent;
   std::string cppContent;
   std::string mathheader;
   std::string mathcppContent;
   std::string ftpheader;
   std::string ftpcppContent;
   bool sourceEmbedded = false;
};

class POSTINTERFACE_SHAREDLIB PostInterface {
public:
    std::map<std::string, TemplateInfo> templates_;
    // 带自动注册功能的构造函数
    explicit PostInterface();
    static PostInterface& getInstance();

    // 注册模板
    void registerTemplate(const std::string& name,
        const std::string& headerPath,
        const std::string& cppPath,
        const std::string& className,
        const std::string& headerContent,
        const std::string& cppContent);

    // 移除模板
    void unregisterTemplate(const std::string& name);

    // 获取所有可用模板
    std::vector<std::string> getAvailableTemplates() const;

    // 获取模板详细信息
    TemplateInfo getTemplateInfo(const std::string& templateName) const;

    // 从DLL加载模板
    bool loadTemplateFromDLL(const std::string& dllPath,
        const std::string& templateName);

    // 自动扫描并加载所有模板
    void autoRegisterTemplates(const std::string& templatesDirectory);

    // 从DLL自动注册模板
    bool autoRegisterFromDLL(const std::string& dllPath);
    bool autoRegisterFromDLLs(const std::string& dllDirectory);

    // 新增功能：复制模板创建新模板
    bool copyTemplate(const std::string& sourceTemplateName,
        const std::string& newTemplateName,
        const std::string& newHeaderPath,
        const std::string& newCppPath);

    // 新增功能：查找模板文件
    bool findTemplateFiles(const std::string& templateName,
        std::string& headerPath,
        std::string& cppPath) const;

    // 析构函数
    ~PostInterface();

private:
    PostInterface(const PostInterface&) = delete;
    PostInterface& operator=(const PostInterface&) = delete;

    // 平台特定的目录遍历函数
    std::vector<std::string> listFilesInDirectory(const std::string & directory, const std::string & extension);
    bool isRegularFile(const std::string & path);
    std::string extractTemplateRegisterFunction(const std::string& content, const std::string& templateName);
    std::string extractClassNameFromFunction(const std::string& content, const std::string& defaultName);
    //动态注册
    
     };
