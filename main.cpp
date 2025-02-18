#include <cassert>
#include <cmath>
#include <format>
#include <iostream>
#include <optional>

enum TypeImpl{
    None,
    Tuple,
    Variadic
};

struct none_type_impl : std::integral_constant<int, TypeImpl::None>{};
struct tuple_type_impl : std::integral_constant<int, TypeImpl::Tuple>{};
struct variadic_type_impl : std::integral_constant<int, TypeImpl::Variadic>{};

template <typename F, typename ...Args>
constexpr none_type_impl count_value(...);

template <typename F, typename ...Args>
constexpr auto count_value(F f) -> decltype( f(), none_type_impl{} );

template <typename F, typename ...Args>
constexpr auto count_value(F f) -> decltype( f(std::declval<std::tuple<Args...>>()), tuple_type_impl{} );

template <typename F, typename T, typename ...Args>
constexpr auto count_value(F f) -> decltype( f(std::declval<T>(), std::declval<Args>()...), variadic_type_impl{} );

template<class If> concept variable = requires(If _if){
    requires(std::is_same_v<If, bool>);
};

template<typename Func1, typename Func2 = decltype([]{})>
class if_run{
public:   
    if_run(Func1 &&func1) : func1_(std::move(func1)), func2_(Func2()){}
    if_run(Func1 &&func1, Func2 &&func2) : func1_(std::move(func1)), func2_(std::move(func2)){}

    if_run(bool _if, Func1 &&func1)
        : if_(_if), func1_(std::move(func1)), func2_(Func2()){}
    if_run(bool _if, Func1 &&func1, Func2 &&func2)
        : if_(_if), func1_(std::move(func1)), func2_(std::move(func2)){}

    template<typename Result, typename ...Args>
    std::optional<Result> operator()(Args &&...args) const{
        if(if_){
            return run_args<Result>(func1_, std::forward<Args>(args)...);
        }
        else{
            return run_args<Result>(func2_, std::forward<Args>(args)...);
        }
        return std::optional<Result>();
    }

    template<typename Result, typename ...Args>
    std::optional<Result> operator()(std::tuple<Args...> &&tuple) const{
        if(if_){
            return run_tuple<Result>(func1_, std::forward<decltype(tuple)>(tuple));
        }
        else{
            return run_tuple<Result>(func2_, std::forward<decltype(tuple)>(tuple));
        }
        return std::optional<Result>();
    }

    template<typename Result, variable If, typename ...Args>
    std::optional<Result> operator()(If _if, Args &&...args){
        if_ = _if;
        return operator()<Result>(std::forward<Args>(args)...);
    }

    template<typename Result, variable If, typename ...Args>
    std::optional<Result> operator()(If _if, std::tuple<Args...> &&tuple){
        if_ = _if;
        return operator()<Result>(std::forward<decltype(tuple)>(tuple));
    }

private:
    bool if_{};

    Func1 func1_;
    Func2 func2_;

    template<typename Result, typename Func, typename ...Args>
    std::optional<Result> run_args(Func &&func, Args &&...args) const{
        if constexpr(decltype(count_value<Func, Args...>(func))::value == TypeImpl::Tuple){
            if constexpr(std::is_same_v<typename std::invoke_result<decltype(func), std::tuple<Args...>>::type, void>){
                func(std::tie<Args...>(args...));
            }
            else{
                return func(std::tie<Args...>(args...));
            }
        }
        else if constexpr(decltype(count_value<Func, Args...>(func))::value == TypeImpl::Variadic){
            if constexpr(std::is_same_v<typename std::invoke_result<decltype(func), Args...>::type, void>){
                func(std::forward<Args>(args)...);
            }
            else{
                return func(std::forward<Args>(args)...);
            }
        }
        else if constexpr(decltype(count_value<Func, Args...>(func))::value == TypeImpl::None){
            if constexpr(std::is_same_v<typename std::invoke_result<decltype(func)>::type, void>){
                func();
            }
            else{
                return func();
            }
        }
        else{
            static_assert(false, "There is no suitable type for this function.");
        }
        return std::optional<Result>();
    }

    template<typename Result,  typename Func, typename ...Args>
    std::optional<Result> run_tuple(Func &&func, std::tuple<Args...> &&tuple) const{
        if constexpr(decltype(count_value<Func, Args...>(func))::value == TypeImpl::Tuple){
            if constexpr(std::is_same_v<typename std::invoke_result<decltype(func), std::tuple<Args...>>::type, void>){
                func(std::forward<decltype(tuple)>(tuple));
            }
            else{
                return func(std::forward<decltype(tuple)>(tuple));
            }
        }
        else if constexpr(decltype(count_value<Func, Args...>(func))::value == TypeImpl::None){
            if constexpr(std::is_same_v<typename std::invoke_result<decltype(func)>::type, void>){
                func();
            }
            else{
                return func();
            }
        }
        else{
            static_assert(false, "There is no suitable type for this function.");
        }
        return std::optional<Result>();
    }
};

int main(){
    int a = 1;
    int b = 4;
    int c = 1;
    try {
        if_run if2([](){
            throw std::logic_error("D < 0");
        }, [](auto &&tuple){
            auto&[a,b,_,D] = tuple;
            return std::pair<double,double>(b - std::sqrt(D) / (2 * a), (b + std::sqrt(D)) / (2 * a));
        });
        auto t = if_run(a == 0, [](){
            throw std::logic_error("a = 0");
        },[&if2](auto &&tuple){
            auto&[a,b,c] = tuple;
            int D = b * b - 4 * a * c;
            return if2.operator()<std::pair<double,double>>(D < 0, std::move(std::tuple_cat(tuple, std::make_tuple(D))));
        }).operator()<std::pair<double,double>>(a,b,c);

        std::cout << std::format("x1 = {}, x2 = {}", t.value().first, t.value().second);
    } catch (const std::exception &e) {
        std::cout << e.what();
        return 0;
    }
    return 0;
}
