// SET CERTAIN EVENTS TO BE TRIGGERED VIA INTERRUPTS TO AVOID RACE CONDITIONS 

// put in namespace telescope?

// i still don't understand why all the function definitions are in the .hpp file 

namespace telescope {

    class Telescope {
        Telescope() : currentState(TelescopeState::INIT) {}

        // put this elsewhere?
        enum class TelescopeState {
            INIT, 
            READY,
            SEARCH,
            FOUND
        };

        // enum or struct bools? im leaning towards struct bools, bc we need to use a combo  
        struct Event {
            bool SelectConfiguration{false};
            bool ConfigurationSelected{false};
            bool ObjectSelected{false};
            bool ObjectNotFound{false};
            bool ObjectFound{false};
            bool RequestCancelled{false};
        };

        void handle_init() {
            // any initialization (not done in PostInit)

            // wait for telescope configuration to be selected

            // set configuration variables 

            // determine current FOV (astro function)

            // determine objects within FOV (astro function)

            // if choice selected, send command to graphics to highlight objects within FOV (astro function) -- way to do this w/o code duplication across states?

            if (ConfigurationSelected) {
                currentState = TelescopeState::READY;
            }
        }

        void handle_ready() {
            // wait for object to be selected

            // set selected object

            // determine current FOV (astro function)

            // determine objects within FOV (astro function)

            // if choice selected, send command to graphics to highlight objects within FOV (astro function) -- way to do this w/o code duplication across states?

            if (ObjectSelected) {
                currentState = TelescopeState::SEARCH;
            }
            
        }

        void handle_search() {
            // determine current FOV (astro function)

            // determine objects within FOV (astro function)

            // if choice selected, send command to graphics to highlight objects within FOV (astro function) -- way to do this w/o code duplication across states?
            
            // determine whether star would be within current FOV 

            if (ObjectFound) {
                currentState = TelescopeState::FOUND;
            }
        }

        void handle_found() {
            // determine current FOV (astro function)

            // determine objects within FOV (astro function)

            // if choice selected, send command to graphics to highlight objects within FOV (astro function) -- way to do this w/o code duplication across states?

            // determine whether star would be within current FOV 

            // if star within current FOV, send command to graphics to highlight that specific target

            // if star out of that FOV, set state back to search

            if (ObjectNotFound) {
                currentState = TelescopeState::SEARCH;
            }
        }
    
    private:
        TelescopeState currentState;
    }

} //namespace telescope 
