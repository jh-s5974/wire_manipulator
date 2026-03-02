#pragma once

#include <memory>
#include <vector>
#include <functional>

namespace project {
    namespace signal {

        template <typename T>
        class Tx;

        template <typename T>
        class Rx {
            std::function<void(const T&)> callback;
            int reference;
            public:
            Rx(std::function<void(const T&)> task): callback(task), reference(0) {}
            void set(std::function<void(const T&)> task) {
                callback = task;
            }

            int ref_count() {return reference;}

            using SharedPtr = std::shared_ptr<Rx<T>>;

            friend class Tx<T>;
        };

        template <typename T>
        class Tx {
            std::vector<std::shared_ptr<Rx<T>>> targets;
        public:
            void connect(std::shared_ptr<Rx<T>>& rx) {
                targets.push_back(rx);
                rx->reference++;
            }

            void operator += (std::shared_ptr<Rx<T>>& rx) {
                connect(rx);
            }
            
            void send(const T& data) {
                for (auto& tgt: targets) {
                    if (tgt)
                        tgt->callback(data);
                }
            }

            int ref_count() {return targets.size();}

            using SharedPtr = std::shared_ptr<Tx<T>>;
        };


        template<>
        class Rx<void> {
            std::function<void(void)> callback;
            int reference;
            public:
            Rx(std::function<void(void)> task): callback(task) {}
            void set(std::function<void()> task) {
                callback = task;
            }

            int ref_count() {return reference;}

            using SharedPtr = std::shared_ptr<Rx<void>>;

            friend class Tx<void>;
        };

        template<>
        class Tx<void> {
            std::vector<std::shared_ptr<Rx<void>>> targets;
        public:
            void connect(std::shared_ptr<Rx<void>>& rx) {
                targets.push_back(rx);
            }

            void operator += (std::shared_ptr<Rx<void>>& rx) {
                connect(rx);
            }
            
            void send() {
                for (auto& tgt: targets) {
                    tgt->callback();
                }
            }

            int ref_count() {return targets.size();}


            using SharedPtr = std::shared_ptr<Tx<void>>;
        };

    };

};