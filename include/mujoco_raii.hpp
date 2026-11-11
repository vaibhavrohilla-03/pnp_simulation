#pragma once
#include <memory>
#include "mujoco/mujoco.h"

namespace mujoco_raii {

struct ModelDeleter {
    void operator()(mjModel* model) const noexcept {
        if (model != nullptr) {
            mj_deleteModel(model);
        }
    }
};

struct DataDeleter {
    void operator()(mjData* data) const noexcept {
        if (data != nullptr) {
            mj_deleteData(data);
        }
    }
};

struct SpecDeleter {
    void operator()(mjSpec* spec) const noexcept {
        if (spec != nullptr) {
            mj_deleteSpec(spec);
        }
    }
};

using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr  = std::unique_ptr<mjData, DataDeleter>;
using SpecPtr  = std::unique_ptr<mjSpec, SpecDeleter>;

}
