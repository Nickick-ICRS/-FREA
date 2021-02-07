#ifndef __DART_PLUGIN_HPP__
#define __DART_PLUGIN_HPP__

class DartPlugin {
public:
    /**
     * @brief Update function, called every timestep
     *
     * @param dt The (dimulation) time since the last update (in s)
     *
     * @param reset Whether to reset the plugin
     */
    virtual void update(double dt, bool reset) = 0;
};

#endif // __DART_PLUGIN_HPP__
