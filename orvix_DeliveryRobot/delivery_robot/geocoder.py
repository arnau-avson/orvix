from geopy.geocoders import Nominatim
from geopy.exc import GeocoderServiceError

from .models import Point

_USER_AGENT = "orvix-delivery-robot/0.1"
_geolocator = Nominatim(user_agent=_USER_AGENT)


class GeocodingError(RuntimeError):
    pass


def geocode(address: str) -> Point:
    try:
        result = _geolocator.geocode(address, timeout=10)
    except GeocoderServiceError as e:
        raise GeocodingError(f"Nominatim service error: {e}") from e

    if result is None:
        raise GeocodingError(f"Address not found: {address!r}")

    return Point(lat=result.latitude, lon=result.longitude)


def reverse_geocode(point: Point) -> str:
    try:
        result = _geolocator.reverse(point.as_tuple(), timeout=10)
    except GeocoderServiceError as e:
        raise GeocodingError(f"Nominatim service error: {e}") from e

    if result is None:
        raise GeocodingError(f"No address for point {point}")

    return result.address
