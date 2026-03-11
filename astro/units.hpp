
// TO DO: specify specific float size 
// TO DO: unify case structure lmao 

struct EquatorialCoordinates {
    float right_ascension;
    float declination;
};

struct HorizontalCoordinates {
    float altitude;
    float azimuth;
};

struct GraphicsCommand {
    string object;
    EquatorialCoordinates position;
};