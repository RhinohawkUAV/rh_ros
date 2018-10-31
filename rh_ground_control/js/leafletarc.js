var _extends = Object.assign || function (target) {
  for (var i = 1; i < arguments.length; i++) {
    var source = arguments[i];

    for (var key in source) {
      if (Object.prototype.hasOwnProperty.call(source, key)) {
        target[key] = source[key];
      }
    }
  }

  return target;
};













var objectWithoutProperties = function (obj, keys) {
  var target = {};

  for (var i in obj) {
    if (keys.indexOf(i) >= 0) continue;
    if (!Object.prototype.hasOwnProperty.call(obj, i)) continue;
    target[i] = obj[i];
  }

  return target;
};

L.Arc = L.Polyline.extend({
    options: {
        weight: 5,
        color: '#ffff00',
        stroke: true
    },

    initialize: function initialize(_ref) {
        var _ref$center = _ref.center,
            center = _ref$center === undefined ? [0, 0] : _ref$center,
            _ref$radius = _ref.radius,
            radius = _ref$radius === undefined ? 100 : _ref$radius,
            _ref$startBearing = _ref.startBearing,
            startBearing = _ref$startBearing === undefined ? 0 : _ref$startBearing,
            _ref$endBearing = _ref.endBearing,
            endBearing = _ref$endBearing === undefined ? 90 : _ref$endBearing,
            _ref$numberOfPoints = _ref.numberOfPoints,
            numberOfPoints = _ref$numberOfPoints === undefined ? 32 : _ref$numberOfPoints,
            _ref$rhumb = _ref.rhumb,
            rhumb = _ref$rhumb === undefined ? false : _ref$rhumb,
            options = objectWithoutProperties(_ref, ['center', 'radius', 'startBearing', 'endBearing', 'numberOfPoints', 'rhumb']);

        this.setOptions(options).setCenter(center).setRadius(radius).setStartBearing(startBearing).setEndBearing(endBearing).setNumberOfPoints(numberOfPoints).setRhumb(rhumb);

        this._setLatLngs(this.getLatLngs());
    },
    getCenter: function getCenter() {
        return this._center;
    },
    setCenter: function setCenter(center) {
        this._center = L.latLng(center);
        return this.redraw();
    },
    getRadius: function getRadius() {
        return this._radius;
    },
    setRadius: function setRadius() {
        var radius = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : 100;

        this._radius = Math.abs(radius);
        return this.redraw();
    },
    getStartBearing: function getStartBearing() {
        return this._startBearing;
    },
    setStartBearing: function setStartBearing() {
        var startBearing = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : 0;

        /**
         * Not sure how much of these checks are neccessary
         * just using all as a temp fix for rotation problems.
         */
        var endBearing = this.getEndBearing() || 360;

        while (startBearing < 0) {
            startBearing += 360;
        }
        while (startBearing > 360) {
            startBearing -= 360;
        }

        if (endBearing < startBearing) {
            while (endBearing <= startBearing) {
                startBearing = startBearing - 360;
            }
        }

        while (endBearing - startBearing > 360) {
            startBearing += 360;
        }this._startBearing = startBearing;
        return this.redraw();
    },
    getEndBearing: function getEndBearing() {
        return this._endBearing;
    },
    setEndBearing: function setEndBearing() {
        var endBearing = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : 90;

        /**
         * Not sure how much of these checks are neccessary
         * just using all as a temp fix for rotation problems.
         */
        var startBearing = this.getStartBearing() || 0;

        while (endBearing < 0) {
            endBearing += 360;
        }
        while (endBearing > 360) {
            endBearing -= 360;
        }

        if (startBearing > endBearing) {
            while (startBearing >= endBearing) {
                endBearing += 360;
            }
        }

        while (endBearing - startBearing > 360) {
            endBearing -= 360;
        }this._endBearing = endBearing;
        return this.redraw();
    },
    getNumberOfPoints: function getNumberOfPoints() {
        return this._numberOfPoints;
    },
    setNumberOfPoints: function setNumberOfPoints() {
        var numberOfPoints = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : 32;

        this._numberOfPoints = Math.max(10, numberOfPoints);
        return this.redraw();
    },
    getOptions: function getOptions() {
        return this.options;
    },
    setOptions: function setOptions() {
        var options = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : {};

        L.setOptions(this, options);
        return this.redraw();
    },
    getLatLngs: function getLatLngs() {
        var angle = this.getEndBearing() - this.getStartBearing();
        var ptCount = angle * this.getNumberOfPoints() / 360;
        var latlngs = [];
        var deltaAngle = angle / ptCount;

        for (var i = 0; i < ptCount; i++) {
            var useAngle = this.getStartBearing() + deltaAngle * i;
            latlngs.push(this.computeDestinationPoint(this.getCenter(), this.getRadius(), useAngle));
        }
        latlngs.push(this.computeDestinationPoint(this.getCenter(), this.getRadius(), this.getEndBearing()));
        return latlngs;
    },
    setLatLngs: function setLatLngs() {
        var latLngs = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : this.getLatLngs();

        this._setLatLngs(latLngs);
        return this.redraw();
    },


    setStyle: L.Path.prototype.setStyle,

    getRhumb: function getRhumb() {
        return this._rhumb;
    },
    setRhumb: function setRhumb() {
        var rhumb = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : 45;

        this._rhumb = rhumb;
        return this.redraw();
    },
    computeDestinationPoint: function computeDestinationPoint() {
        var start = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : { lat: 0, lng: 0 };
        var distance = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : 1;
        var bearing = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : 0;
        var radius = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : 6378137;
        var rhumb = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : this.getRhumb();

        if (rhumb) {
            /*http://www.movable-type.co.uk/scripts/latlong.html*/

            var δ = Number(distance) / radius; // angular distance in radians
            var φ1 = start.lat * Math.PI / 180;
            var λ1 = start.lng * Math.PI / 180;
            var θ = bearing * Math.PI / 180;

            var Δφ = δ * Math.cos(θ);
            var φ2 = φ1 + Δφ;

            // check for some daft bugger going past the pole, normalise latitude if so
            if (Math.abs(φ2) > Math.PI / 2) φ2 = φ2 > 0 ? Math.PI - φ2 : -Math.PI - φ2;

            var Δψ = Math.log(Math.tan(φ2 / 2 + Math.PI / 4) / Math.tan(φ1 / 2 + Math.PI / 4));
            var q = Math.abs(Δψ) > 10e-12 ? Δφ / Δψ : Math.cos(φ1); // E-W course becomes ill-conditioned with 0/0

            var Δλ = δ * Math.sin(θ) / q;
            var λ2 = λ1 + Δλ;

            //return new LatLon(φ2.toDegrees(), (λ2.toDegrees()+540) % 360 - 180); // normalise to −180..+180°
            return {
                lat: φ2 * 180 / Math.PI,
                lng: (λ2 * 180 / Math.PI + 540) % 360 - 180
            };
        }
        var bng = bearing * Math.PI / 180;

        var lat1 = start.lat * Math.PI / 180;
        var lon1 = start.lng * Math.PI / 180;

        var lat2 = Math.asin(Math.sin(lat1) * Math.cos(distance / radius) + Math.cos(lat1) * Math.sin(distance / radius) * Math.cos(bng));

        var lon2 = lon1 + Math.atan2(Math.sin(bng) * Math.sin(distance / radius) * Math.cos(lat1), Math.cos(distance / radius) - Math.sin(lat1) * Math.sin(lat2));

        lat2 = lat2 * 180 / Math.PI;
        lon2 = lon2 * 180 / Math.PI;

        return {
            lat: lat2,
            lng: lon2
        };
    }
});

L.arc = function (_ref2) {
    var _ref2$center = _ref2.center,
        center = _ref2$center === undefined ? [0, 0] : _ref2$center,
        _ref2$radius = _ref2.radius,
        radius = _ref2$radius === undefined ? 100 : _ref2$radius,
        _ref2$startBearing = _ref2.startBearing,
        startBearing = _ref2$startBearing === undefined ? 0 : _ref2$startBearing,
        _ref2$endBearing = _ref2.endBearing,
        endBearing = _ref2$endBearing === undefined ? 90 : _ref2$endBearing,
        _ref2$numberOfPoints = _ref2.numberOfPoints,
        numberOfPoints = _ref2$numberOfPoints === undefined ? 32 : _ref2$numberOfPoints,
        _ref2$rhumb = _ref2.rhumb,
        rhumb = _ref2$rhumb === undefined ? false : _ref2$rhumb,
        options = objectWithoutProperties(_ref2, ['center', 'radius', 'startBearing', 'endBearing', 'numberOfPoints', 'rhumb']);
    return new L.Arc(_extends({ center: center, radius: radius, rhumb: rhumb, startBearing: startBearing, numberOfPoints: numberOfPoints, endBearing: endBearing }, options));
};