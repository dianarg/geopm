#include <string>

class SingletonName
{
    public:
        ~SingletonName();
        std::string name(void);
        void name(std::string nn);
        static SingletonName &singleton_name(void);
    private:
        SingletonName();
        std::string m_name;
};
